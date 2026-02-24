#include "easynav_multirobot_exploration/DetectFrontier.hpp"

namespace multirobot_exploration
{

DetectFrontier::DetectFrontier(
    const std::string& name,
    const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
  // Retrieve the shared ROS node and namespace prefix from the blackboard
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_prefix_ = config().blackboard->get<std::string>("tf_prefix");
  RCLCPP_INFO(node_->get_logger(), "** DetectFrontier **");

  // Create publisher for visual debugging of the frontier in RViz
  frontier_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "frontier_marker", rclcpp::QoS(10).transient_local().reliable());
}

BT::NodeStatus
DetectFrontier::tick()
{
  OccupancyGrid::SharedPtr grid;
  // Ensure the map is available on the blackboard before proceeding
  if (!getInput("map", grid) || !grid) {
    RCLCPP_ERROR(node_->get_logger(), "No map");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose robot_pose;
  // Ensure the robot's current pose is available on the blackboard
  if (!getInput("robot_pose", robot_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "No robot pose");
    return BT::NodeStatus::FAILURE;
  }
  
  // Execute the core OpenCV frontier extraction algorithm
  std::vector<geometry_msgs::msg::Point> frontier;
  frontier = get_frontier(grid, robot_pose);

  // If the map is fully explored or the algorithm fails, return FAILURE to trigger alternative BT logic
  if (frontier.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to detect frontier");
    return BT::NodeStatus::FAILURE;
  }
  
  // Package the raw coordinates into a ROS Marker and publish them
  auto marker = fill_marker(frontier);
  marker.header.frame_id = grid->header.frame_id;

  frontier_pub_->publish(marker);
  
  // Expose the valid frontier points to the rest of the behavior tree
  setOutput("frontier", frontier);

  RCLCPP_INFO(
    node_->get_logger(),
    "Frontier generated with size %ld",
    frontier.size()
  );
  return BT::NodeStatus::SUCCESS;
}

visualization_msgs::msg::Marker
DetectFrontier::fill_marker(
  std::vector<geometry_msgs::msg::Point> frontier)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = node_->now();
  marker.ns = "frontier";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::POINTS;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  // Points size
  marker.scale.x = 0.05; 
  marker.scale.y = 0.05;

  // Color (Default: Solid Blue)
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  // Append each geometric point to the marker's array
  for (const auto& pt : frontier) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0; // Flat 2D exploration
    marker.points.push_back(p);
  }
  return marker;
}

std::vector<geometry_msgs::msg::Point>
DetectFrontier::get_frontier(
  OccupancyGrid::SharedPtr map,
  geometry_msgs::msg::Pose& pose)
{
  int width = map->info.width;
  int height = map->info.height;
  double res = map->info.resolution;
  double ox = map->info.origin.position.x;
  double oy = map->info.origin.position.y;

  // Convert real-world meter coordinates (x, y) to grid indices (col, row)
  int pose_x = static_cast<int>((pose.position.x - ox) / res);
  int pose_y = static_cast<int>((pose.position.y - oy) / res);

  // Safety check: Abort if the robot is outside the map bounds or outside free space
  if (pose_x < 0 || pose_x >= width || pose_y < 0 || pose_y >= height) {
    RCLCPP_ERROR(node_->get_logger(), "Robot not in map");
    return {};
  }
  if (map->data[pose_y * width + pose_x] != 0) {
    RCLCPP_ERROR(node_->get_logger(), "Robot not in free space");
    return {};
  }

  cv::Mat map_mat(height, width, CV_8SC1, const_cast<int8_t*>(map->data.data()));
  
  // Split map in 3 pure binary masks
  cv::Mat free_space = (map_mat == 0);
  cv::Mat unknown_space = (map_mat == -1);
  cv::Mat obstacles = (map_mat == 100);


  // Simulate a "paint bucket" originating from the robot to find all connected free space
  cv::Rect rect;
  cv::Mat reachable_mask = cv::Mat::zeros(height + 2, width + 2, CV_8UC1);

  cv::floodFill(free_space, reachable_mask, cv::Point(pose_x, pose_y), cv::Scalar(128),
                &rect, cv::Scalar(0), cv::Scalar(0), 4 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);
  cv::Mat reachable_actual = reachable_mask(cv::Rect(1, 1, width, height));


  // Dilate and intersect unknowk space 
  // Guarantees resulting pixels to belong to safe, navigable 0-cost areas.
  cv::Mat frontiers;
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

  cv::dilate(unknown_space, frontiers, kernel);
  cv::bitwise_and(frontiers, reachable_actual, frontiers);


  // Eliminate frontier pixels close to obstacle pixels
  // Avoid frontier to be too close to obstacles
  cv::Mat danger_zone;
  cv::Mat wall_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(9, 9));
  cv::dilate(obstacles, danger_zone, wall_kernel);

  cv::Mat safe_zone;
  cv::bitwise_not(danger_zone, safe_zone);
  cv::bitwise_and(frontiers, safe_zone, frontiers);


  // Eliminate frontier pixels surrounding isolated unknown pixels
  // Reduce noise and clean frontier
  cv::Mat isolated_centers;
  cv::Mat pixels_to_delete;

  cv::morphologyEx(frontiers, isolated_centers, cv::MORPH_HITMISS, CROSS_KERNEL);
  cv::dilate(isolated_centers, pixels_to_delete, CROSS_MASK);
  cv::bitwise_and(frontiers, ~pixels_to_delete, frontiers);


  // Extract the non-zero pixel coordinates into a vector safely
  std::vector<cv::Point> raw_points;
  if (cv::countNonZero(frontiers) > 0) {
    cv::findNonZero(frontiers, raw_points);
  }

  // Translate the local OpenCV grid indices back into global ROS spatial coordinates (meters)
  std::vector<geometry_msgs::msg::Point> frontier_points;
  frontier_points.reserve(raw_points.size());

  for (const auto& pt : raw_points) {
    geometry_msgs::msg::Point target_point;
    // Shift by res/2 to point precisely to the center of the grid cell
    target_point.x = ox + (pt.x * res) + (res / 2.0);
    target_point.y = oy + (pt.y * res) + (res / 2.0);
    target_point.z = 0.0;
    frontier_points.push_back(target_point);
  }

  return frontier_points;
}

} // namespace multirobot_exploration


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::DetectFrontier>("DetectFrontier");
}
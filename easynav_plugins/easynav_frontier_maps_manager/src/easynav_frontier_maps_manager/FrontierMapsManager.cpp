#include "easynav_frontier_maps_manager/FrontierMapsManager.hpp"


namespace easynav
{

using namespace std::chrono_literals;
using std::placeholders::_1;

FrontierMapsManager::FrontierMapsManager()
{
  // NavState::register_printer<Costmap2D>(
  //   [](const Costmap2D & map) {
  //     std::ostringstream oss;
  //     oss << "Costmap2D of (" << map.getSizeInCellsX() << " x " << map.getSizeInCellsY()
  //         << ") with resolution " << map.getResolution();
  //     return oss.str();
  //   });
}

FrontierMapsManager::~FrontierMapsManager() {}

std::expected<void, std::string>
FrontierMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  RCLCPP_INFO(node->get_logger(), "Loading Frontier Maps Manager");

  // Create publisher for visual debugging of the frontier in RViz
  frontier_pub_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/points",
    rclcpp::QoS(10).transient_local().reliable()
  );

  return {};
}

void
FrontierMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  // Wait until we have robot position
  if (!nav_state.has("robot_pose")) {
    return;
  }
  const auto& robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");

  
  // Wait until we have map
  const auto& fixed_costmap = nav_state.get<Costmap2D>("map.static");
  
  if (fixed_costmap.getSizeInCellsX() == 0 || fixed_costmap.getSizeInCellsY() == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "No map yet");
    return;
  }

  OccupancyGrid fixed_map;
  fixed_costmap.toOccupancyGridMsg(fixed_map);


  // Execute the core OpenCV frontier extraction algorithm
  std::vector<geometry_msgs::msg::Point> frontier;
  frontier = get_frontier(fixed_map, robot_pose);

  // If the map is fully explored or the algorithm fails, return FAILURE to trigger alternative BT logic
  if (frontier.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "Unable to detect frontier");
    return;
  }
  
  // Package the raw coordinates into a ROS Marker and publish them
  auto marker = fill_marker(frontier);
  marker.header.frame_id = get_tf_prefix() + "map";
  frontier_pub_->publish(marker);
  
  // Expose the valid frontier points to the rest of the behavior tree
  nav_state.set("frontier", frontier);

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Frontier generated with size %ld",
    frontier.size()
  );
}

visualization_msgs::msg::Marker
FrontierMapsManager::fill_marker(
  std::vector<geometry_msgs::msg::Point> frontier)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = get_node()->now();
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
FrontierMapsManager::get_frontier(
  OccupancyGrid map,
  const nav_msgs::msg::Odometry& pose)
{
  int width = map.info.width;
  int height = map.info.height;
  double res = map.info.resolution;
  double ox = map.info.origin.position.x;
  double oy = map.info.origin.position.y;

  // Convert real-world meter coordinates (x, y) to grid indices (col, row)
  int pose_x = static_cast<int>((pose.pose.pose.position.x - ox) / res);
  int pose_y = static_cast<int>((pose.pose.pose.position.y - oy) / res);

  // Safety check: Abort if the robot is outside the map bounds or outside free space
  if (pose_x < 0 || pose_x >= width || pose_y < 0 || pose_y >= height) {
    RCLCPP_ERROR(get_node()->get_logger(), "Robot not in map");
    return {};
  }
  if (map.data[pose_y * width + pose_x] != 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Robot not in free space");
    return {};
  }

  cv::Mat map_mat(height, width, CV_8SC1, const_cast<int8_t*>(map.data.data()));
  
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

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::FrontierMapsManager, easynav::MapsManagerBase)

#include "easynav_multirobot_exploration/DetectFrontier.hpp"

namespace multirobot_exploration
{

DetectFrontier::DetectFrontier(
    const std::string& name,
    const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_prefix_ = config().blackboard->get<std::string>("tf_prefix");
  RCLCPP_INFO(node_->get_logger(), "** DetectFrontier **");

  frontier_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "frontier_marker", rclcpp::QoS(10).transient_local().reliable());

  int noise_kernel_type, dilate_kernel_type;
  node_->declare_parameter("noise_kernel_size", 0);
  node_->declare_parameter("noise_kernel_type", 0);
  node_->declare_parameter("dilate_kernel_type", 0);

  node_->get_parameter("noise_kernel_size", noise_kernel_size_);
  node_->get_parameter("noise_kernel_type", noise_kernel_type);
  node_->get_parameter("dilate_kernel_type", dilate_kernel_type);

  switch (noise_kernel_type) {
  case RECT:
    noise_kernel_type_ = cv::MORPH_RECT;
    break;
  case CROSS:
    noise_kernel_type_ = cv::MORPH_CROSS;
    break;
  case ELLIPSE:
    noise_kernel_type_ = cv::MORPH_ELLIPSE;
  }

  switch (dilate_kernel_type) {
  case RECT:
    dilate_kernel_type_ = cv::MORPH_RECT;
    break;
  case CROSS:
    dilate_kernel_type_ = cv::MORPH_CROSS;
    break;
  case ELLIPSE:
    dilate_kernel_type_ = cv::MORPH_ELLIPSE;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Noise kernel size: %dx%d, type: %d",
    noise_kernel_size_, noise_kernel_size_, noise_kernel_type
  );
  RCLCPP_INFO(
    node_->get_logger(), "Dilate kernel type: %d",
    dilate_kernel_type
  );
}

BT::NodeStatus
DetectFrontier::tick()
{
  OccupancyGrid::SharedPtr grid;
  if (!getInput("map", grid) || !grid) {
    RCLCPP_ERROR(node_->get_logger(), "No map");
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::Pose robot_pose;
  if (!getInput("robot_pose", robot_pose)) {
    RCLCPP_ERROR(node_->get_logger(), "No robot pose");
    return BT::NodeStatus::FAILURE;
  }
  
  std::vector<geometry_msgs::msg::Point> frontier;
  frontier = get_frontier(grid, robot_pose);

  if (frontier.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to detect frontier");
    return BT::NodeStatus::FAILURE;
  }
  
  auto marker = fill_marker(frontier);
  marker.header.frame_id = grid->header.frame_id;

  frontier_pub_->publish(marker);
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
  
  // Tamaño
  marker.scale.x = 0.05; 
  marker.scale.y = 0.05;

  // Color
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  for (const auto& pt : frontier) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;
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

  int pose_x = static_cast<int>((pose.position.x - ox) / res);
  int pose_y = static_cast<int>((pose.position.y - oy) / res);

  if (pose_x < 0 || pose_x >= width || pose_y < 0 || pose_y >= height) {
    RCLCPP_ERROR(node_->get_logger(), "Robot not in map");
    return {};
  }
  if (map->data[pose_y * width + pose_x] != 0) {
    RCLCPP_ERROR(node_->get_logger(), "Robot not in free space");
    return {};
  }

  cv::Mat map_mat(height, width, CV_8SC1, const_cast<int8_t*>(map->data.data()));
  cv::Mat free_space = (map_mat == 0);
  cv::Mat unknown_space = (map_mat == -1);

  cv::Mat map_clean_kernel = cv::getStructuringElement(
    noise_kernel_type_, cv::Size(noise_kernel_size_, noise_kernel_size_));
  
  // cv::morphologyEx(free_space, free_space, cv::MORPH_OPEN, map_clean_kernel);
  // cv::morphologyEx(free_space, free_space, cv::MORPH_CLOSE, map_clean_kernel);
  cv::morphologyEx(unknown_space, unknown_space, cv::MORPH_OPEN, map_clean_kernel);
  // cv::morphologyEx(unknown_space, unknown_space, cv::MORPH_CLOSE, map_clean_kernel);

  cv::Mat reachable_mask = cv::Mat::zeros(height + 2, width + 2, CV_8UC1);
  cv::Rect rect;

  cv::floodFill(free_space, reachable_mask, cv::Point(pose_x, pose_y), cv::Scalar(128),
                &rect, cv::Scalar(0), cv::Scalar(0), 4 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);

  cv::Mat reachable_actual = reachable_mask(cv::Rect(1, 1, width, height));

  cv::Mat frontiers;
  cv::Mat kernel = cv::getStructuringElement(dilate_kernel_type_, cv::Size(3, 3));
  cv::dilate(unknown_space, frontiers, kernel);
  cv::bitwise_and(frontiers, reachable_actual, frontiers);

  std::vector<cv::Point> raw_points;
  if (cv::countNonZero(frontiers) > 0) {
    cv::findNonZero(frontiers, raw_points);
  }

  std::vector<geometry_msgs::msg::Point> frontier_points;
  frontier_points.reserve(raw_points.size());

  for (const auto& pt : raw_points) {
    geometry_msgs::msg::Point target_point;
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
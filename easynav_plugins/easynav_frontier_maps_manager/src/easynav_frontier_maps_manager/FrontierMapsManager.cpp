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

void
FrontierMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  RCLCPP_INFO(node->get_logger(), "Loading Frontier Maps Manager");

  // Get frontier params
  node->declare_parameter(plugin_name + ".proximity_radius", 0.0);
  node->declare_parameter(plugin_name + ".obstacle_threshold", easynav::FREE_SPACE);

  node->get_parameter(plugin_name + ".proximity_radius", proximity_radius_);
  node->get_parameter(plugin_name + ".obstacle_threshold", obstacle_threshold_);

  // Create publisher for visual debugging of the frontier in RViz
  frontier_pub_ = get_node()->create_publisher<visualization_msgs::msg::Marker>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/points",
    rclcpp::QoS(10).transient_local().reliable()
  );
}

void
FrontierMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  // Wait until we have robot position
  if (!nav_state.has("robot_pose") || !nav_state.has("map.dynamic")) return;

  const auto& robot_pose = nav_state.get<nav_msgs::msg::Odometry>("robot_pose");
  const auto& fixed_map = nav_state.get<Costmap2D>("map.dynamic");

  // Wait until we have map
  if (fixed_map.getSizeInCellsX() == 0 || fixed_map.getSizeInCellsY() == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "No map yet");
    return;
  }

  // Execute the core OpenCV frontier extraction algorithm
  std::vector<geometry_msgs::msg::Point> frontier;
  frontier = get_frontier(fixed_map, robot_pose);

  // If the map is fully explored or the algorithm fails, return FAILURE to trigger alternative BT logic
  if (frontier.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "No frontier possible");
    return;
  }
  
  // Package the raw coordinates into a ROS Marker and publish them
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  rclcpp::Time map_stamp = nav_state.get<rclcpp::Time>("map_time");

  auto marker = fill_marker(frontier);
  marker.header.frame_id = tf_info.map_frame;
  marker.header.stamp = map_stamp;
  frontier_pub_->publish(marker);
  
  // Expose the valid frontier points to the rest of the easynav system
  nav_state.set("frontier", frontier);
}

visualization_msgs::msg::Marker
FrontierMapsManager::fill_marker(
  const std::vector<geometry_msgs::msg::Point>& frontier)
{
  visualization_msgs::msg::Marker marker;
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
  const Costmap2D& map,
  const nav_msgs::msg::Odometry& pose)
{
 // Obtian costmap data
  int width = map.getSizeInCellsX();
  int height = map.getSizeInCellsY();
  double res = map.getResolution();
  double ox = map.getOriginX();
  double oy = map.getOriginY();

  // Localization verification before initilazing frontier search
  unsigned int pose_x, pose_y;
  if (!map.worldToMap(pose.pose.pose.position.x, pose.pose.pose.position.y, pose_x, pose_y)) {
    RCLCPP_WARN(get_node()->get_logger(), "Robot not in map.");
    return {};
  }
  unsigned char robot_cost = map.getCost(pose_x, pose_y);
  if (robot_cost < easynav::FREE_SPACE || robot_cost > obstacle_threshold_) {
      RCLCPP_WARN(
        get_node()->get_logger(), 
        "Robot is not in free space (Cost: %d)", 
        robot_cost
      );
      return {};
  }

  /** Frontier search */
  cv::Mat map_mat(height, width, CV_8UC1, map.getCharMap());
  free_space_ = (map_mat >= easynav::FREE_SPACE) & (map_mat <= obstacle_threshold_);
  unknown_space_ = (map_mat == easynav::NO_INFORMATION);


  // Resize only if neccesary
  if (reachable_mask_.size() != cv::Size(width + 2, height + 2)) {
    reachable_mask_ = cv::Mat::zeros(height + 2, width + 2, CV_8UC1);
  } else {
    reachable_mask_.setTo(0);
  }

  // Simulate a "paint bucket" originating from the robot to find all connected free space
  cv::Rect rect;
  cv::floodFill(free_space_, reachable_mask_, cv::Point(pose_x, pose_y), cv::Scalar(128),
                &rect, cv::Scalar(0), cv::Scalar(0), 4 | (255 << 8) | cv::FLOODFILL_MASK_ONLY);
  reachable_actual_ = reachable_mask_(cv::Rect(1, 1, width, height));


  // Intersect with dilated unknown space, to get total frontier 
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));

  cv::dilate(unknown_space_, frontiers_, kernel);
  cv::bitwise_and(frontiers_, reachable_actual_, frontiers_);


  // Clean frontier points
  cv::Mat isolated_centers;
  cv::Mat pixels_to_delete;

  cv::morphologyEx(frontiers_, isolated_centers, cv::MORPH_HITMISS, CROSS_KERNEL);
  cv::dilate(isolated_centers, pixels_to_delete, CROSS_MASK);
  cv::bitwise_and(frontiers_, ~pixels_to_delete, frontiers_);


  // Extract the non-zero pixel coordinates into a vector safely
  std::vector<cv::Point> raw_points;
  if (cv::countNonZero(frontiers_) > 0) {
    cv::findNonZero(frontiers_, raw_points);
  }

  // Translate the local OpenCV grid indices back into global ROS spatial coordinates (meters)
  std::vector<geometry_msgs::msg::Point> frontier_points;
  frontier_points.reserve(raw_points.size());

  for (const auto& pt : raw_points) {
    // Get distances
    double wx = ox + (pt.x * res) + (res / 2.0);
    double wy = oy + (pt.y * res) + (res / 2.0);

    // Simple proximity filter
    double dx = wx - pose.pose.pose.position.x;
    double dy = wy - pose.pose.pose.position.y;
    if ((dx*dx + dy*dy) < proximity_radius_ * proximity_radius_) continue;

    // Safe final frontier points
    geometry_msgs::msg::Point target_point;
    target_point.x = wx;
    target_point.y = wy;
    target_point.z = 0.0;
    frontier_points.push_back(target_point);
  }

  return frontier_points;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::FrontierMapsManager, easynav::MapsManagerBase)

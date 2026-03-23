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

  // Frontier extraction params
  node->declare_parameter(plugin_name + ".obstacle_threshold", easynav::FREE_SPACE);
  node->declare_parameter(plugin_name + ".proximity_radius", 0.0);
  node->declare_parameter(plugin_name + ".clustering", false);

  node->get_parameter(plugin_name + ".obstacle_threshold", obstacle_threshold_);
  node->get_parameter(plugin_name + ".proximity_radius", proximity_radius_);
  node->get_parameter(plugin_name + ".clustering", clustering_);

  if (clustering_) {
    RCLCPP_INFO(node->get_logger(), "Clustering mode active");

    node->declare_parameter(plugin_name + ".dbscan_eps_px", 0);
    node->declare_parameter(plugin_name + ".dbscan_min_points", 0);

    node->get_parameter(plugin_name + ".dbscan_eps_px", dbscan_eps_px_);
    node->get_parameter(plugin_name + ".dbscan_min_points", dbscan_min_points_);

  } else  RCLCPP_INFO(node->get_logger(), "Raw mode active");


  // Init internal marker parameters
  marker_.ns = "frontier";
  marker_.id = 0;
  marker_.action = visualization_msgs::msg::Marker::ADD;

  marker_.color.r = 0.0;
  marker_.color.g = 0.0;
  marker_.color.b = 1.0; // Solid blue
  marker_.color.a = 1.0;


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
  
  // Wait until we have map
  const auto& fixed_map = nav_state.get<Costmap2D>("map.dynamic");
  if (fixed_map.getSizeInCellsX() == 0 || fixed_map.getSizeInCellsY() == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "No map yet");
    return;
  }

  // Execute the core OpenCV frontier extraction algorithm
  std::vector<geometry_msgs::msg::Point> frontier;
  frontier = get_frontier(fixed_map, robot_pose);

  // Invalid != empty, empty may be useful
  if (invalid_frontier_) {
    RCLCPP_WARN(get_node()->get_logger(), "Invalid frontier");

    // Reset, by default frontiers are supossed to be valid
    invalid_frontier_ = false;
    return;
  }

  // Publish and save result in navstate
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  rclcpp::Time map_stamp = nav_state.get<rclcpp::Time>("map_time");

  fill_marker(frontier);
  marker_.header.frame_id = tf_info.map_frame;
  marker_.header.stamp = map_stamp;
  frontier_pub_->publish(marker_);
  
  // Expose the valid frontier points to the rest of the easynav system
  nav_state.set("frontier", frontier);
}

void
FrontierMapsManager::fill_marker(
  const std::vector<geometry_msgs::msg::Point>& points)
{
  // Select points size
  if (clustering_) {
    marker_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker_.scale.x = 0.15; 
    marker_.scale.y = 0.15;
    marker_.scale.z = 0.15;
  } else {
    marker_.type = visualization_msgs::msg::Marker::POINTS;
    marker_.scale.x = 0.05; 
    marker_.scale.y = 0.05;
  }

  // Append each geometric point to the marker's array
  marker_.points.clear();
  for (const auto& pt : points) {
    geometry_msgs::msg::Point p;
    p.x = pt.x;
    p.y = pt.y;
    p.z = 0.0;
    marker_.points.push_back(p);
  }
}

std::vector<cv::Point>
FrontierMapsManager::get_centroids_DBSCAN(cv::Mat& points)
{
  // Epsilon: dilate to group points
  cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,
    cv::Size(2 * dbscan_eps_px_ + 1, 2 * dbscan_eps_px_ + 1)
  );

  cv::Mat clustered;
  cv::dilate(points, clustered, kernel);

  // Apply clustering
  cv::Mat labels, stats, raw_centroids;
  int n_labels = cv::connectedComponentsWithStats(clustered, labels, stats, raw_centroids);


  // Extract centroids
  std::vector<cv::Point> centroids;
  centroids.reserve(n_labels);

  for (int i = 1; i < n_labels; i++) {
    // Discard insignificant groups with min_points
    if (stats.at<int>(i, cv::CC_STAT_AREA) < dbscan_min_points_) continue;

    // We need centroid to be ON the frontier
    cv::Point2d math_center(raw_centroids.at<double>(i, 0), raw_centroids.at<double>(i, 1));

    int left = stats.at<int>(i, cv::CC_STAT_LEFT);
    int top = stats.at<int>(i, cv::CC_STAT_TOP);
    int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
    int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

    cv::Point best_p;
    bool found = false;
    double min_dist = std::numeric_limits<double>::max();

    // Get closest point to centroid ON the frontier
    for (int y = top; y < top + height; ++y) {
      for (int x = left; x < left + width; ++x) {
        if (labels.at<int>(y, x) == i && points.at<uchar>(y, x) > 0) {
          double dist = cv::norm(cv::Point2d(x, y) - math_center);
          
          if (dist < min_dist) {
            min_dist = dist;
            best_p = cv::Point(x, y);
            found = true;
    }}}}
  
    if (found) centroids.push_back(best_p);
  }

  return centroids;
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
    invalid_frontier_ = true;
    return {};
  }

  unsigned char robot_cost = map.getCost(pose_x, pose_y);
  if (robot_cost < easynav::FREE_SPACE || robot_cost > obstacle_threshold_) {
    RCLCPP_WARN(
      get_node()->get_logger(), 
      "Robot is not in free space (Cost: %d)", 
      robot_cost
    );
    invalid_frontier_ = true;
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
  cv::Mat frontier;

  cv::dilate(unknown_space_, frontier, kernel);
  cv::bitwise_and(frontier, reachable_actual_, frontier);


  // Extract frontier points,
  // grouping them in clusters or using raw points
  std::vector<cv::Point> frontier_points;
  bool centroids_empty = false;

  if (clustering_) {
    frontier_points = get_centroids_DBSCAN(frontier);
    if (frontier_points.empty()) centroids_empty = true;

  } else if (cv::countNonZero(frontier) > 0) {
    cv::findNonZero(frontier, frontier_points);
  }

  // Translate to ROS msg
  std::vector<geometry_msgs::msg::Point> final_frontier;
  final_frontier.reserve(frontier_points.size());

  for (const auto& fc: frontier_points) {
    double wx = ox + (fc.x * res) + (res / 2.0);
    double wy = oy + (fc.y * res) + (res / 2.0);
  
    // Proximity security filter
    double dx = wx - pose.pose.pose.position.x;
    double dy = wy - pose.pose.pose.position.y;
    if ((dx*dx + dy*dy) < proximity_radius_*proximity_radius_) continue;

    // Save to final list
    geometry_msgs::msg::Point p;
    p.x = wx;
    p.y = wy;
    p.z = 0.0;
    final_frontier.push_back(p);
  }

  // If clustering wasnt possible due to proximity filter, change to raw mode
  if (clustering_) {
    invalid_frontier_ = final_frontier.empty() && !centroids_empty;

    if (invalid_frontier_) { // Only when centroid(s) where eliminated due to proximity filter
      clustering_ = false;
      RCLCPP_INFO(get_node()->get_logger(),
        "No DBSCAN clustering possible, changing to Raw Frontier mode.");

      // Timer to re-try clustering in x seconds
      if (!clustering_timer_) {
        clustering_timer_ = get_node()->create_wall_timer(
          5s, [this]() {
            clustering_ = true;
            RCLCPP_INFO(get_node()->get_logger(), "Re-Trying Clustering");
          }
        );
      }

    // When cluster possible, delete timer
    } else {clustering_timer_ = nullptr;}
  }

  return final_frontier;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::FrontierMapsManager, easynav::MapsManagerBase)

#include "easynav_multiplexor_maps_manager/MultiplexorMapsManager.hpp"


namespace easynav
{

using namespace std::chrono_literals;
using std::placeholders::_1;

MultiplexorMapsManager::MultiplexorMapsManager()
{
  // NavState::register_printer<Costmap2D>(
  //   [](const Costmap2D & map) {
  //     std::ostringstream oss;
  //     oss << "Costmap2D of (" << map.getSizeInCellsX() << " x " << map.getSizeInCellsY()
  //         << ") with resolution " << map.getResolution();
  //     return oss.str();
  //   });
}

MultiplexorMapsManager::~MultiplexorMapsManager() {}

std::expected<void, std::string>
MultiplexorMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  RCLCPP_INFO(node->get_logger(), "Loading Multiplexor Maps Manager");

  // Get the list of active robot identifiers from the parameters
  std::vector<std::string> namespaces;
  node->declare_parameter(plugin_name + ".robot_namespaces", namespaces);
  node->get_parameter(plugin_name + ".robot_namespaces", namespaces);

  RCLCPP_INFO(node->get_logger(), "Maps to merge: %ld", namespaces.size());

  // Establish the local robot's frame as the global coordinate origin (0,0).
  std::string fixed_map_ns;
  node->declare_parameter(plugin_name + ".fixed_map_ns", fixed_map_ns);
  node->get_parameter(plugin_name + ".fixed_map_ns", fixed_map_ns);

  RCLCPP_INFO(node->get_logger(), "Fixed map ns: %s", fixed_map_ns.c_str());

  // Initialize coordinates for each robot and create map subscriptions.
  float x, y, Y;

  for (const auto& ns : namespaces) {
    std::string param_prefix = plugin_name + "." + ns;
    std::string x_key = param_prefix + ".x";
    std::string y_key = param_prefix + ".y";
    std::string Y_key = param_prefix + ".Y";

    // Declare and get the initial pose (x, y, yaw) for each robot.
    node->declare_parameter(x_key, 0.0);
    node->declare_parameter(y_key, 0.0);
    node->declare_parameter(Y_key, 0.0);

    node->get_parameter(x_key, x);
    node->get_parameter(y_key, y);
    node->get_parameter(Y_key, Y);

    // Store the initial pose in the local cache.
    robots_coords_[ns].x = x;
    robots_coords_[ns].y = y;
    robots_coords_[ns].theta = Y;

    // Do not suscribe to main map (is in navstate)
    if (ns == fixed_map_ns) {continue;}
  
    // Create a subscription to the map topic for this specific robot.
    // Using transient_local QoS ensures we receive the latest map ("latched" behavior).
    std::string topic_name;
    node->declare_parameter(param_prefix + ".topic", topic_name);
    node->get_parameter(param_prefix + ".topic", topic_name);

    map_subs_[ns] = node->create_subscription<OccupancyGrid>(
        "/" + ns + "/" + topic_name,
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&MultiplexorMapsManager::map_callback, this, _1)
    );
  }
  translate_robot_coords(fixed_map_ns);

  // Publisher for the final merged global map.
  muxed_map_pub_ = node->create_publisher<OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name + "/map",
    rclcpp::QoS(1).transient_local().reliable()
  );

  return {};
}

void
MultiplexorMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  // Wait until there is map
  const auto& fixed_costmap = nav_state.get<Costmap2D>("map.static");
  
  if (fixed_costmap.getSizeInCellsX() == 0 || fixed_costmap.getSizeInCellsY() == 0) {
    RCLCPP_DEBUG(get_node()->get_logger(), "No map yet");
    return;
  }

  // When map arrives, multiplex maps over received fixed map
  OccupancyGrid muxed_map, fixed_map;

  fixed_costmap.toOccupancyGridMsg(fixed_map);
  mux(fixed_map, muxed_map);

  // Update static map
  // // Esto es el objetivo
  // // pero tendría que modificar el CostmapMapsManager
  // nav_state.set("map.static") = muxed_map;

  // Publish map for visualization
  muxed_map.header.frame_id = get_tf_prefix() + "map";
  muxed_map.header.stamp = get_node()->now();
  muxed_map_pub_->publish(muxed_map);
}

void
MultiplexorMapsManager::map_callback(const OccupancyGrid::SharedPtr map)
{
  // Extract the robot ID from the frame_id string (e.g., "r1/map" -> "r1").
  std::string frame = map->header.frame_id;
  size_t pos = frame.find('/');
  std::string ns = (pos != std::string::npos) ? frame.substr(0, pos) : frame;

  // Cache the received map.
  maps_[ns] = map;
}
void
MultiplexorMapsManager::translate_robot_coords(std::string fixed_ns)
{
  // Verify the reference robot exists in our coordinate list.
  if (robots_coords_.find(fixed_ns) == robots_coords_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(),
              "%s not in robot list", fixed_ns.c_str());
    return; 
  }

  // Retrieve the pose of the reference robot to use as the new origin.
  geometry_msgs::msg::Pose2D origin = robots_coords_[fixed_ns];
  double ox = origin.x;
  double oy = origin.y;
  double oth = origin.theta;

  // Precompute trigonometric values for rotation.
  double c = std::cos(oth);
  double s = std::sin(oth);

  std::map<std::string, geometry_msgs::msg::Pose2D> coords_transformed;
  
  // Transform all other robots' coordinates relative to this new origin.
  for (const auto& [id, p] : robots_coords_) {
      geometry_msgs::msg::Pose2D p_new;

      // Translate position relative to origin.
      double dx = p.x - ox;
      double dy = p.y - oy;

      // Rotate position to align with origin's orientation.
      p_new.x = (dx * c) + (dy * s);
      p_new.y = -(dx * s) + (dy * c);

      // Adjust orientation relative to origin's yaw.
      p_new.theta = p.theta - oth;
      // Normalize angle to [-PI, PI].
      p_new.theta = std::atan2(std::sin(p_new.theta), std::cos(p_new.theta));

      coords_transformed[id] = p_new;
  }

  // Overwrite the internal coordinate list with the transformed values.
  robots_coords_ = coords_transformed;
}

BoundingBox
MultiplexorMapsManager::get_global_bounds()
{
  BoundingBox box;

  // Iterate through all available maps to find the global min/max coordinates.
  for (const auto& [id, map] : maps_) {
    if (robots_coords_.find(id) == robots_coords_.end()) continue;

    geometry_msgs::msg::Pose2D pose = robots_coords_[id];
    double c = std::cos(pose.theta);
    double s = std::sin(pose.theta);

    // Calculate map dimensions in meters.
    double w = map->info.width * map->info.resolution;
    double h = map->info.height * map->info.resolution;
    double ox = map->info.origin.position.x;
    double oy = map->info.origin.position.y;

    // Define the four corners of the local map.
    double local_corners_x[4] = {ox, ox + w, ox + w, ox};
    double local_corners_y[4] = {oy, oy, oy + h, oy + h};

    // Transform each corner to the global frame and update bounding box.
    for (int i = 0; i < 4; i++) {
      double gx = pose.x + (local_corners_x[i] * c - local_corners_y[i] * s);
      double gy = pose.y + (local_corners_x[i] * s + local_corners_y[i] * c);

      if (gx < box.min_x) box.min_x = gx;
      if (gx > box.max_x) box.max_x = gx;
      if (gy < box.min_y) box.min_y = gy;
      if (gy > box.max_y) box.max_y = gy;
    }
  }

  // Handle case where no valid bounds were found.
  if (box.min_x > box.max_x) {
      box.min_x = box.min_y = box.max_x = box.max_y = 0.0;
  }

  // Add a safety margin (PADDING) around the calculated bounds.
  box.min_x -= PADDING;
  box.min_y -= PADDING;
  box.max_x += PADDING;
  box.max_y += PADDING;

  return box;
}

void
MultiplexorMapsManager::mux(const OccupancyGrid& src, OccupancyGrid& dst)
{
  if (src.info.width == 0 || src.info.height == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Src map is empty, skipping mux");
    return;
  }

  double res = src.info.resolution;
  double local_x = src.info.origin.position.x;
  double local_y = src.info.origin.position.y;
  int w = src.info.width;
  int h = src.info.height;
  
  // Calculate padding required to expand the local map to the global bounds.
  BoundingBox bounds = get_global_bounds();

  int pad_south = std::max(0, static_cast<int>(std::round((local_y - bounds.min_y) / res)));
  int pad_north = std::max(0, static_cast<int>(std::round((bounds.max_y - (local_y + h * res)) / res)));
  int pad_west  = std::max(0, static_cast<int>(std::round((local_x - bounds.min_x) / res)));
  int pad_east  = std::max(0, static_cast<int>(std::round((bounds.max_x - (local_x + w * res)) / res)));

  // Note: data is treated as signed 8-bit integers (-1, 0, 100).
  cv::Mat dst_mat;
  cv::Mat src_mat(h, w, CV_8SC1, reinterpret_cast<int8_t*>(const_cast<signed char*>(src.data.data())));

  // Expand the canvas using borders filled with -1 (unknown space).
  cv::copyMakeBorder(src_mat, dst_mat, 
                     pad_south, pad_north, pad_west, pad_east, 
                     cv::BORDER_CONSTANT, cv::Scalar(-1));
  
  // Calculate the new origin of the expanded map.
  double origin_x = local_x - (pad_west * res);
  double origin_y = local_y - (pad_south * res);

  // Buffer for affine transformation.
  cv::Mat warped;
  cv::Point2f src_points[3], dst_points[3];

  // Merge other robots' maps into the expanded canvas.
  for (const auto& [ns, map] : maps_) {
    if (!map) continue;
    if (robots_coords_.find(ns) == robots_coords_.end()) continue;

    geometry_msgs::msg::Pose2D pose = robots_coords_[ns];
    cv::Mat extra_mat(map->info.height, map->info.width, CV_8SC1, reinterpret_cast<int8_t*>(map->data.data()));

    // Define 3 points (origin, top-right, bottom-left) to compute the affine transform.
    src_points[0] = {0.f, 0.f};
    src_points[1] = {static_cast<float>(map->info.width), 0.f};
    src_points[2] = {0.f, static_cast<float>(map->info.height)};

    float c = std::cos(pose.theta);
    float s = std::sin(pose.theta);
    float ox = map->info.origin.position.x;
    float oy = map->info.origin.position.y;
    float r = map->info.resolution;

    // Transform the 3 source points to the destination pixel coordinates on the global canvas.
    for(int i = 0; i < 3; ++i) {
        float mx = ox + src_points[i].x * r;
        float my = oy + src_points[i].y * r;
        // Apply rotation and translation.
        float wx = pose.x + (mx * c - my * s);
        float wy = pose.y + (mx * s + my * c);
        // Convert back to grid indices relative to new origin.
        dst_points[i].x = (wx - origin_x) / res;
        dst_points[i].y = (wy - origin_y) / res;
    }

    // Apply the affine warp to align the remote map with the global frame.
    cv::Mat M = cv::getAffineTransform(src_points, dst_points);
    cv::warpAffine(extra_mat, warped, M, dst_mat.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(-1));
    
    // Merge logic: keep max value (e.g., Obstacle(100) > Free(0) > Unknown(-1)).
    // This ensures obstacles are preserved even if another map sees them as free or unknown.
    cv::max(dst_mat, warped, dst_mat);
  }

  // Populate the final OccupancyGrid message.
  dst.header = src.header;
  dst.header.stamp = get_node()->now();
  dst.info.resolution = res;
  dst.info.width = dst_mat.cols;
  dst.info.height = dst_mat.rows;
  dst.info.origin.position.x = origin_x;
  dst.info.origin.position.y = origin_y;
  dst.info.origin.orientation = src.info.origin.orientation;

  // Copy raw data from the OpenCV matrix to the ROS message vector.
  size_t size = dst_mat.total() * dst_mat.elemSize();
  dst.data.resize(size);
  std::memcpy(dst.data.data(), dst_mat.data, size);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MultiplexorMapsManager, easynav::MapsManagerBase)

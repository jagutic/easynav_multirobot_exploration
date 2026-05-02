#include "easynav_multiplexor_maps_manager/MultiplexorMapsManager.hpp"

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

namespace easynav
{

using namespace std::chrono_literals;
using std::placeholders::_1;

MultiplexorMapsManager::MultiplexorMapsManager()
{
  // NavState::register_printer<Costmap2D>(
  //   [](const Costmap2D & map) {
  //     std::ostringstream oss;
  //     oss << "Costmap2D of (" << map.getSizeInCellsX() << " x " <<
  //     map.getSizeInCellsY()
  //         << ") with resolution " << map.getResolution();
  //     return oss.str();
  //   });
}

MultiplexorMapsManager::~MultiplexorMapsManager() {}

void
MultiplexorMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  RCLCPP_INFO(node->get_logger(), "Loading Multiplexor Maps Manager");

  // Initialize the TF broadcaster with its neccesary node in global ns
  global_tf_node_ = std::make_shared<rclcpp::Node>(
    "mux_global_tf_broadcaster", "/",
    rclcpp::NodeOptions().use_global_arguments(false));

  global_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(global_tf_node_);

  // Get the list of active robot identifiers from the parameters
  // And get the actual robot to work on
  std::vector<std::string> robot_namespaces;

  node->declare_parameter(plugin_name + ".robot_namespaces", robot_namespaces);
  node->get_parameter(plugin_name + ".robot_namespaces", robot_namespaces);
  RCLCPP_INFO(node->get_logger(), "Maps to merge: %ld", robot_namespaces.size());

  node->declare_parameter(plugin_name + ".fixed_map_ns", fixed_map_ns_);
  node->get_parameter(plugin_name + ".fixed_map_ns", fixed_map_ns_);
  RCLCPP_INFO(node->get_logger(), "Fixed map ns: %s", fixed_map_ns_.c_str());

  // Initialize coordinates for each robot and create map subscriptions.
  float x, y, Y;

  for (const auto & ns : robot_namespaces) {
    std::string param_prefix = plugin_name + "." + ns;
    std::string x_key = param_prefix + ".x";
    std::string y_key = param_prefix + ".y";
    std::string Y_key = param_prefix + ".Y";
    std::string topic_key = param_prefix + ".topic";

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

    // Crear suscriptor y publicar TF usando las funciones refactorizadas
    create_map_subscriber(ns, topic_key);
  }
  
  // Create tf from global frame to local map frame
  create_static_tf(fixed_map_ns_);

  // Establish the local robot's frame as the global coordinate origin (0,0).
  translate_robot_coords(fixed_map_ns_);

  muxed_map_pub_ = node->create_publisher<OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name +
        "/map",
    rclcpp::QoS(1).transient_local().reliable());
}

void
MultiplexorMapsManager::create_map_subscriber(const std::string & ns, const std::string & topic_key)
{
  std::string topic_name;
  
  get_node()->declare_parameter(topic_key, topic_name);
  get_node()->get_parameter(topic_key, topic_name);

  // Create a subscription to the map topic for this specific robot.
  // Using transient_local QoS ensures we receive the latest map ("latched"
  // behavior).
  map_subs_[ns] = get_node()->create_subscription<OccupancyGrid>(
      "/" + ns + "/" + topic_name,
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MultiplexorMapsManager::map_callback, this, _1));

  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to map: /%s/%s", ns.c_str(), topic_name.c_str());
}

void
MultiplexorMapsManager::create_static_tf(const std::string & ns)
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  
  // Retrieve the stored coordinates for this robot
  geometry_msgs::msg::Pose2D pose = robots_coords_[ns];

  // Create the static transform message
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_node()->get_clock()->now();
  t.header.frame_id = GLOBAL_MAP_FRAME;
  t.child_frame_id = tf_info.map_frame;

  // Set translation
  t.transform.translation.x = pose.x;
  t.transform.translation.y = pose.y;
  t.transform.translation.z = 0.0;

  // Convert yaw angle to quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, pose.theta);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Publish the static transform
  global_tf_broadcaster_->sendTransform(t);

  RCLCPP_INFO(get_node()->get_logger(),
    "Published static transform: %s -> %s with translation (%.2f, %.2f) and rotation (%.2f rad)",
    t.header.frame_id.c_str(), t.child_frame_id.c_str(), pose.x, pose.y, pose.theta);
}

void
MultiplexorMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  // Mux all maps on fixed costmap and save in muxed map
  Costmap2D muxed_map;

  std::unique_lock<std::mutex> lock(maps_mutex_);
  mux(muxed_map);
  lock.unlock();

  // Save new map in navstate so it can be used
  nav_state.set("map.static", muxed_map);
  nav_state.set("map.static.update", true);

  // Publish map for visualization
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  rclcpp::Time map_stamp = nav_state.get<rclcpp::Time>("map_time");

  OccupancyGrid muxed_map_msg;
  muxed_map.toOccupancyGridMsg(muxed_map_msg);

  muxed_map_msg.header.frame_id = tf_info.map_frame;
  muxed_map_msg.header.stamp = map_stamp;
  muxed_map_pub_->publish(muxed_map_msg);
}

void
MultiplexorMapsManager::map_callback(const OccupancyGrid::SharedPtr map)
{
  // Extract the robot ID from the frame_id string (e.g., "r1/map" -> "r1").
  std::string frame = map->header.frame_id;
  size_t pos = frame.find('/');
  std::string ns = (pos != std::string::npos) ? frame.substr(0, pos) : frame;

  // Cache the received map.
  std::unique_lock<std::mutex> lock(maps_mutex_);
  maps_[ns] = Costmap2D(*map);
  lock.unlock();
}

void
MultiplexorMapsManager::translate_robot_coords(std::string fixed_ns)
{
  // Verify the reference robot exists in our coordinate list.
  if (robots_coords_.find(fixed_ns) == robots_coords_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s not in robot list",
                 fixed_ns.c_str());
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
  for (const auto &[id, p] : robots_coords_) {
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
  for (const auto &[id, map] : maps_) {
    if (map.getSizeInCellsX() == 0 || map.getSizeInCellsY() == 0) {continue;}

    geometry_msgs::msg::Pose2D pose = robots_coords_[id];
    double c = std::cos(pose.theta);
    double s = std::sin(pose.theta);

    // Calculate map dimensions in meters.
    double w = map.getSizeInMetersX();
    double h = map.getSizeInMetersY();
    double ox = map.getOriginX();
    double oy = map.getOriginY();

    // Define the four corners of the local map.
    double local_corners_x[4] = {ox, ox + w, ox + w, ox};
    double local_corners_y[4] = {oy, oy, oy + h, oy + h};

    // Transform each corner to the global frame and update bounding box.
    for (int i = 0; i < 4; i++) {
      double gx = pose.x + (local_corners_x[i] * c - local_corners_y[i] * s);
      double gy = pose.y + (local_corners_x[i] * s + local_corners_y[i] * c);

      if (gx < box.min_x) {
        box.min_x = gx;
      }
      if (gx > box.max_x) {
        box.max_x = gx;
      }
      if (gy < box.min_y) {
        box.min_y = gy;
      }
      if (gy > box.max_y) {
        box.max_y = gy;
      }
    }
  }

  // Handle case where no valid bounds were found.
  if (box.min_x > box.max_x) {
    box.min_x = box.min_y = box.max_x = box.max_y = 0.0;
  }

  return box;
}

void
MultiplexorMapsManager::mux(Costmap2D & dst)
{

  // Get fixed map from the list
  const Costmap2D & fixed_map = maps_[fixed_map_ns_];
  if (fixed_map.getSizeInCellsX() == 0 || fixed_map.getSizeInCellsY() == 0) {
    RCLCPP_WARN(get_node()->get_logger(), "Not fixed map yet");
    return;
  }

  // Get fixed map data
  double res = fixed_map.getResolution();
  double local_x = fixed_map.getOriginX();
  double local_y = fixed_map.getOriginY();
  unsigned int w = fixed_map.getSizeInCellsX();
  unsigned int h = fixed_map.getSizeInCellsY();

  // Resize final costmap and create aux final mat filled with NO_INFORMATION
  BoundingBox bounds = get_global_bounds();
  double aligned_ox = local_x - std::floor((local_x - bounds.min_x) / res) * res;
  double aligned_oy = local_y - std::floor((local_y - bounds.min_y) / res) * res;

  uint32_t new_w = std::ceil((bounds.max_x - aligned_ox) / res);
  uint32_t new_h = std::ceil((bounds.max_y - aligned_oy) / res);

  dst.resizeMap(new_w, new_h, res, aligned_ox, aligned_oy);
  dst.resetMapToValue(0, 0, new_w, new_h, easynav::NO_INFORMATION);
  cv::Mat dst_mat(new_h, new_w, CV_8UC1, dst.getCharMap());

  // Copy fixed mat to dst mat on origin position
  int offset_x = std::round((local_x - dst.getOriginX()) / res);
  int offset_y = std::round((local_y - dst.getOriginY()) / res);

  cv::Mat fixed_mat(h, w, CV_8UC1, fixed_map.getCharMap());
  fixed_mat.copyTo(dst_mat(cv::Rect(offset_x, offset_y, w, h)));

  // Buffer for affine transformation.
  cv::Mat warped;
  cv::Point2f src_points[3], dst_points[3];

  // Merge other robots maps into the expanded mat.
  for (const auto &[ns, incoming_map] : maps_) {
    if (ns == fixed_map_ns_) {continue;}
    if (incoming_map.getSizeInCellsX() == 0 || incoming_map.getSizeInCellsY() == 0) {continue;}

    // Use costmap for data correspondecy
    cv::Mat incoming_mat(incoming_map.getSizeInCellsY(),
      incoming_map.getSizeInCellsX(), CV_8UC1,
      incoming_map.getCharMap());
    geometry_msgs::msg::Pose2D pose = robots_coords_[ns];

    // Define 3 points (origin, top-right, bottom-left) to compute the affine
    // transform.
    src_points[0] = {0.f, 0.f};
    src_points[1] = {static_cast<float>(incoming_map.getSizeInCellsX()),
      0.f};
    src_points[2] = {0.f,
      static_cast<float>(incoming_map.getSizeInCellsY())};

    float c = std::cos(pose.theta);
    float s = std::sin(pose.theta);
    float ox = incoming_map.getOriginX();
    float oy = incoming_map.getOriginY();
    float r = incoming_map.getResolution();

    // Transform the 3 source points to the destination pixel coordinates on the
    // global canvas.
    for (int i = 0; i < 3; ++i) {
      float mx = ox + src_points[i].x * r;
      float my = oy + src_points[i].y * r;

      // Apply rotation and translation.
      float wx = pose.x + (mx * c - my * s);
      float wy = pose.y + (mx * s + my * c);

      // Convert back to grid indices relative to new origin.
      dst_points[i].x = (wx - dst.getOriginX()) / res;
      dst_points[i].y = (wy - dst.getOriginY()) / res;
    }

    // Apply the affine warp to align the remote map with the global frame.
    cv::Mat M = cv::getAffineTransform(src_points, dst_points);
    cv::warpAffine(incoming_mat, warped, M, dst_mat.size(), cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT, cv::Scalar(easynav::NO_INFORMATION));

    // Copy valid data only if we dont have info about that space
    cv::Mat valid_mask = (dst_mat == easynav::NO_INFORMATION) &
      (warped != easynav::NO_INFORMATION);
    warped.copyTo(dst_mat, valid_mask);
  }
}

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MultiplexorMapsManager,
                       easynav::MapsManagerBase)
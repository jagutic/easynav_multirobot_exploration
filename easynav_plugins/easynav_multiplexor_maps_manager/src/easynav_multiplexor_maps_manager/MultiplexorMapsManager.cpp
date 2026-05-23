#include "easynav_multiplexor_maps_manager/MultiplexorMapsManager.hpp"

namespace easynav
{

using namespace std::chrono_literals;
using std::placeholders::_1;

MultiplexorMapsManager::MultiplexorMapsManager()
{
}

MultiplexorMapsManager::~MultiplexorMapsManager() {}

void
MultiplexorMapsManager::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();
  RCLCPP_INFO(node->get_logger(), "Loading Multiplexor Maps Manager");

  global_tf_broadcaster_node_ = std::make_shared<rclcpp::Node>(
    "global_tf_broadcaster_node", "/",
    rclcpp::NodeOptions().use_global_arguments(false));
    
  global_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(global_tf_broadcaster_node_);
  global_static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(global_tf_broadcaster_node_);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  std::vector<std::string> robot_namespaces;

  node->declare_parameter(plugin_name + ".robot_namespaces", robot_namespaces);
  node->get_parameter(plugin_name + ".robot_namespaces", robot_namespaces);
  RCLCPP_INFO(node->get_logger(), "Maps to merge: %ld", robot_namespaces.size());

  node->declare_parameter(plugin_name + ".fixed_map_ns", fixed_map_ns_);
  node->get_parameter(plugin_name + ".fixed_map_ns", fixed_map_ns_);
  RCLCPP_INFO(node->get_logger(), "Fixed map ns: %s", fixed_map_ns_.c_str());

  float x, y, Y;

  for (const auto & ns : robot_namespaces) {
    std::string param_prefix = plugin_name + "." + ns;
    std::string x_key = param_prefix + ".x";
    std::string y_key = param_prefix + ".y";
    std::string Y_key = param_prefix + ".Y";
    std::string topic_key = param_prefix + ".topic";

    node->declare_parameter(x_key, 0.0);
    node->declare_parameter(y_key, 0.0);
    node->declare_parameter(Y_key, 0.0);

    node->get_parameter(x_key, x);
    node->get_parameter(y_key, y);
    node->get_parameter(Y_key, Y);

    // Migración de Pose2D a Pose
    robots_coords_[ns].position.x = x;
    robots_coords_[ns].position.y = y;
    robots_coords_[ns].position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, Y);
    robots_coords_[ns].orientation = tf2::toMsg(q);

    create_map_subscriber(ns, topic_key);
  }
  
  muxed_map_pub_ = node->create_publisher<OccupancyGrid>(
    node->get_fully_qualified_name() + std::string("/") + plugin_name +
    "/map",
    rclcpp::QoS(1).transient_local().reliable());

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  create_global_tf(GLOBAL_MAP_FRAME, tf_info.map_frame, robots_coords_[fixed_map_ns_], true);
  
  translate_robot_coords(fixed_map_ns_);
}

void
MultiplexorMapsManager::create_map_subscriber(const std::string & ns, const std::string & topic_key)
{
  std::string topic_name;
  
  get_node()->declare_parameter(topic_key, topic_name);
  get_node()->get_parameter(topic_key, topic_name);

  map_subs_[ns] = get_node()->create_subscription<OccupancyGrid>(
      "/" + ns + "/" + topic_name,
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MultiplexorMapsManager::map_callback, this, _1));

  RCLCPP_INFO(get_node()->get_logger(), "Subscribed to map: /%s/%s", ns.c_str(), topic_name.c_str());
}

void
MultiplexorMapsManager::create_global_tf(
  const std::string& parent, const std::string& child,
  geometry_msgs::msg::Pose& pose, bool static_tf = false)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_node()->get_clock()->now();
  t.header.frame_id = parent;
  t.child_frame_id = child;

  t.transform.translation.x = pose.position.x;
  t.transform.translation.y = pose.position.y;
  t.transform.translation.z = pose.position.z;

  t.transform.rotation = pose.orientation;

  if (static_tf) {
    global_static_tf_broadcaster_->sendTransform(t);
  } else {
    global_tf_broadcaster_->sendTransform(t);
  }

  RCLCPP_DEBUG(get_node()->get_logger(),
    "Published transform: %s -> %s with translation (%.2f, %.2f) and rotation (%.2f rad)",
    t.header.frame_id.c_str(), t.child_frame_id.c_str(), 
    pose.position.x, pose.position.y, tf2::getYaw(pose.orientation));
}

void
MultiplexorMapsManager::update(NavState & nav_state)
{
  EASYNAV_TRACE_EVENT;

  // Set base map in BB
  mux(muxed_map_);

  muxed_map_.touch();
  nav_state.set("map.base", muxed_map_);

  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();
  rclcpp::Time map_stamp = nav_state.get<rclcpp::Time>("map_time");

  OccupancyGrid muxed_map_msg;
  muxed_map_.toOccupancyGridMsg(muxed_map_msg);

  muxed_map_msg.header.frame_id = tf_info.map_frame;
  muxed_map_msg.header.stamp = map_stamp;
  muxed_map_pub_->publish(muxed_map_msg);

  try {
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::TransformStamped tf_msg;

    tf_msg = tf_buffer_->lookupTransform(tf_info.map_frame, tf_info.robot_frame, tf2::TimePointZero);
    
    pose.position.x = tf_msg.transform.translation.x;
    pose.position.y = tf_msg.transform.translation.y;
    pose.position.z = tf_msg.transform.translation.z;
    pose.orientation = tf_msg.transform.rotation;

    create_global_tf(tf_info.map_frame, tf_info.robot_frame, pose, false);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_node()->get_logger(), "TF failed: %s", ex.what());
  }
}

void
MultiplexorMapsManager::map_callback(const OccupancyGrid::SharedPtr map)
{
  std::string frame = map->header.frame_id;
  size_t pos = frame.find('/');
  std::string ns = (pos != std::string::npos) ? frame.substr(0, pos) : frame;

  maps_[ns] = Costmap2D(*map);
}

void
MultiplexorMapsManager::translate_robot_coords(std::string fixed_ns)
{
  if (robots_coords_.find(fixed_ns) == robots_coords_.end()) {
    RCLCPP_ERROR(get_node()->get_logger(), "%s not in robot list",
                 fixed_ns.c_str());
    return;
  }

  geometry_msgs::msg::Pose origin = robots_coords_[fixed_ns];
  double ox = origin.position.x;
  double oy = origin.position.y;
  double oth = tf2::getYaw(origin.orientation);

  double c = std::cos(oth);
  double s = std::sin(oth);

  std::map<std::string, geometry_msgs::msg::Pose> coords_transformed;

  for (const auto &[id, p] : robots_coords_) {
    geometry_msgs::msg::Pose p_new;

    double dx = p.position.x - ox;
    double dy = p.position.y - oy;

    p_new.position.x = (dx * c) + (dy * s);
    p_new.position.y = -(dx * s) + (dy * c);
    p_new.position.z = 0.0;

    double p_theta = tf2::getYaw(p.orientation);
    double new_theta = p_theta - oth;
    new_theta = std::atan2(std::sin(new_theta), std::cos(new_theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, new_theta);
    p_new.orientation = tf2::toMsg(q);

    coords_transformed[id] = p_new;
  }

  robots_coords_ = coords_transformed;
}

BoundingBox
MultiplexorMapsManager::get_bounds()
{
  BoundingBox box;

  for (const auto &[id, map] : maps_) {
    if (map.getSizeInCellsX() == 0 || map.getSizeInCellsY() == 0) {continue;}

    geometry_msgs::msg::Pose pose = robots_coords_[id];
    double theta = tf2::getYaw(pose.orientation);
    double c = std::cos(theta);
    double s = std::sin(theta);

    double w = map.getSizeInMetersX();
    double h = map.getSizeInMetersY();
    double ox = map.getOriginX();
    double oy = map.getOriginY();

    double local_corners_x[4] = {ox, ox + w, ox + w, ox};
    double local_corners_y[4] = {oy, oy, oy + h, oy + h};

    for (int i = 0; i < 4; i++) {
      double gx = pose.position.x + (local_corners_x[i] * c - local_corners_y[i] * s);
      double gy = pose.position.y + (local_corners_x[i] * s + local_corners_y[i] * c);

      if (gx < box.min_x) {box.min_x = gx;}
      if (gx > box.max_x) {box.max_x = gx;}
      if (gy < box.min_y) {box.min_y = gy;}
      if (gy > box.max_y) {box.max_y = gy;}
    }
  }

  if (box.min_x > box.max_x) {
    box.min_x = box.min_y = box.max_x = box.max_y = 0.0;
  }

  return box;
}

void
MultiplexorMapsManager::mux(Costmap2D & dst)
{
  const Costmap2D & fixed_map = maps_[fixed_map_ns_];
  if (fixed_map.getSizeInCellsX() == 0 || fixed_map.getSizeInCellsY() == 0) {
    RCLCPP_WARN(get_node()->get_logger(), "Not fixed map yet");
    return;
  }

  double res = fixed_map.getResolution();
  double local_x = fixed_map.getOriginX();
  double local_y = fixed_map.getOriginY();
  unsigned int w = fixed_map.getSizeInCellsX();
  unsigned int h = fixed_map.getSizeInCellsY();

  BoundingBox bounds = get_bounds();
  double aligned_ox = local_x - std::floor((local_x - bounds.min_x) / res) * res;
  double aligned_oy = local_y - std::floor((local_y - bounds.min_y) / res) * res;

  uint32_t new_w = std::ceil((bounds.max_x - aligned_ox) / res);
  uint32_t new_h = std::ceil((bounds.max_y - aligned_oy) / res);

  dst.resizeMap(new_w, new_h, res, aligned_ox, aligned_oy);
  dst.resetMapToValue(0, 0, new_w, new_h, easynav::NO_INFORMATION);
  cv::Mat dst_mat(new_h, new_w, CV_8UC1, dst.getCharMap());

  int offset_x = std::round((local_x - dst.getOriginX()) / res);
  int offset_y = std::round((local_y - dst.getOriginY()) / res);

  cv::Mat fixed_mat(h, w, CV_8UC1, fixed_map.getCharMap());
  fixed_mat.copyTo(dst_mat(cv::Rect(offset_x, offset_y, w, h)));

  cv::Mat warped;
  cv::Point2f src_points[3], dst_points[3];

  for (const auto &[ns, incoming_map] : maps_) {
    if (ns == fixed_map_ns_) {continue;}
    if (incoming_map.getSizeInCellsX() == 0 || incoming_map.getSizeInCellsY() == 0) {continue;}

    cv::Mat incoming_mat(incoming_map.getSizeInCellsY(),
      incoming_map.getSizeInCellsX(), CV_8UC1,
      incoming_map.getCharMap());
      
    geometry_msgs::msg::Pose pose = robots_coords_[ns];
    double theta = tf2::getYaw(pose.orientation);

    src_points[0] = {0.f, 0.f};
    src_points[1] = {static_cast<float>(incoming_map.getSizeInCellsX()), 0.f};
    src_points[2] = {0.f, static_cast<float>(incoming_map.getSizeInCellsY())};

    float c = std::cos(theta);
    float s = std::sin(theta);
    float ox = incoming_map.getOriginX();
    float oy = incoming_map.getOriginY();
    float r = incoming_map.getResolution();

    for (int i = 0; i < 3; ++i) {
      float mx = ox + src_points[i].x * r;
      float my = oy + src_points[i].y * r;

      float wx = pose.position.x + (mx * c - my * s);
      float wy = pose.position.y + (mx * s + my * c);

      dst_points[i].x = (wx - dst.getOriginX()) / res;
      dst_points[i].y = (wy - dst.getOriginY()) / res;
    }

    cv::Mat M = cv::getAffineTransform(src_points, dst_points);
    cv::warpAffine(incoming_mat, warped, M, dst_mat.size(), cv::INTER_NEAREST,
                   cv::BORDER_CONSTANT, cv::Scalar(easynav::NO_INFORMATION));

    cv::Mat valid_mask = (dst_mat == easynav::NO_INFORMATION) &
      (warped != easynav::NO_INFORMATION);
    warped.copyTo(dst_mat, valid_mask);
  }
}

} // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::MultiplexorMapsManager,
                       easynav::MapsManagerBase)
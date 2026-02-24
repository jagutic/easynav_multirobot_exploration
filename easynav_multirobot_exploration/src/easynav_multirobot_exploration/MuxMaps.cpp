#include "easynav_multirobot_exploration/MuxMaps.hpp"

namespace multirobot_exploration
{

using namespace std::chrono_literals;

MuxMaps::MuxMaps(
  const std::string& name,
  const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_prefix_ = config().blackboard->get<std::string>("tf_prefix");
  RCLCPP_INFO(node_->get_logger(), "** MuxMap **");

  // Retrieve the list of active robot identifiers from the parameter server.
  std::vector<std::string> ids;
  node_->declare_parameter("robot_ids", ids);
  node_->get_parameter("robot_ids", ids);
  RCLCPP_INFO(node_->get_logger(), "Total robots: %ld", ids.size());

  // Initialize coordinates for each robot and create map subscriptions.
  for (const auto& id : ids) {
    std::string x_key = id + ".x";
    std::string y_key = id + ".y";
    std::string Y_key = id + ".Y";

    // Declare and retrieve the initial pose (x, y, yaw) for each robot.
    node_->declare_parameter(x_key, 0.0);
    node_->declare_parameter(y_key, 0.0);
    node_->declare_parameter(Y_key, 0.0);

    float x, y, Y;
    node_->get_parameter(x_key, x);
    node_->get_parameter(y_key, y);
    node_->get_parameter(Y_key, Y);

    // Store the initial pose in the local cache.
    robots_coords_[id].x = x;
    robots_coords_[id].y = y;
    robots_coords_[id].theta = Y;

    // Create a subscription to the map topic for this specific robot.
    // Using transient_local QoS ensures we receive the latest map ("latched" behavior).
    std::string topic_name = "/" + id + "/map";
    map_subs_[id] = node_->create_subscription<OccupancyGrid>(
        topic_name,
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&MuxMaps::map_callback, this, std::placeholders::_1)
    );
  }

  // Publisher for the final merged global map.
  muxed_map_pub_ = node_->create_publisher<OccupancyGrid>(
    "muxed_map", rclcpp::QoS(1).transient_local().reliable());

  // Establish the local robot's frame as the global coordinate origin (0,0).
  RCLCPP_INFO(node_->get_logger(), "Origin set at robot %s", tf_prefix_.c_str());
  translate_robot_coords(tf_prefix_);
}

BT::NodeStatus
MuxMaps::tick()
{
  auto muxed_map = std::make_shared<OccupancyGrid>();
  
  // Execute the map merging process.
  mux(muxed_map);

  // Validate the resulting map dimensions.
  if (muxed_map->info.width == 0 || muxed_map->info.height == 0) {
    RCLCPP_ERROR(node_->get_logger(), "Unable to mux maps");
    return BT::NodeStatus::FAILURE;
  }

  // Update the blackboard and publish the map to ROS.
  setOutput("muxed_map", muxed_map);
  muxed_map_pub_->publish(*muxed_map);

  RCLCPP_INFO(node_->get_logger(), "\t");
  RCLCPP_INFO(
    node_->get_logger(),
    "Muxed Map generated with size %dx%d",
    muxed_map->info.height, muxed_map->info.width
  );
  return BT::NodeStatus::SUCCESS;
}

void
MuxMaps::map_callback(const OccupancyGrid::SharedPtr map)
{
  // Extract the robot ID from the frame_id string (e.g., "r1/map" -> "r1").
  std::string frame = map->header.frame_id;
  size_t pos = frame.find('/');
  std::string id = (pos != std::string::npos) ? frame.substr(0, pos) : frame;

  // Cache the received map.
  maps_[id] = map;
}

void
MuxMaps::translate_robot_coords(std::string origin_coord_id)
{
  // Verify the reference robot exists in our coordinate list.
  if (robots_coords_.find(origin_coord_id) == robots_coords_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "%s not in robot list", origin_coord_id.c_str());
    return; 
  }

  // Retrieve the pose of the reference robot to use as the new origin.
  geometry_msgs::msg::Pose2D origin = robots_coords_[origin_coord_id];
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
      RCLCPP_INFO(node_->get_logger(),
          "ID: %s, X: %.2f, Y: %.2f, theta: %.2f",
          id.c_str(), p_new.x, p_new.y, p_new.theta);
  }

  // Overwrite the internal coordinate list with the transformed values.
  robots_coords_ = coords_transformed;
}

BoundingBox
MuxMaps::get_global_bounds()
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
MuxMaps::mux(OccupancyGrid::SharedPtr final_map)
{
  // Ensure the local robot's map is available before proceeding.
  if (maps_.find(tf_prefix_) == maps_.end()) {
    RCLCPP_ERROR(node_->get_logger(), "%s map not published yet", tf_prefix_.c_str());
    return;
  }

  OccupancyGrid::SharedPtr local_map = maps_[tf_prefix_];
  BoundingBox bounds = get_global_bounds();

  double res = local_map->info.resolution;
  double local_x = local_map->info.origin.position.x;
  double local_y = local_map->info.origin.position.y;
  int w = local_map->info.width;
  int h = local_map->info.height;

  // Calculate padding required to expand the local map to the global bounds.
  int pad_south = std::max(0, static_cast<int>(std::round((local_y - bounds.min_y) / res)));
  int pad_north = std::max(0, static_cast<int>(std::round((bounds.max_y - (local_y + h * res)) / res)));
  int pad_west  = std::max(0, static_cast<int>(std::round((local_x - bounds.min_x) / res)));
  int pad_east  = std::max(0, static_cast<int>(std::round((bounds.max_x - (local_x + w * res)) / res)));

  // Create the base matrix using the local map's data.
  cv::Mat total_mat;
  // Note: local_map data is treated as signed 8-bit integers (-1, 0, 100).
  cv::Mat local_mat(h, w, CV_8SC1, reinterpret_cast<int8_t*>(local_map->data.data()));

  // Expand the canvas using borders filled with -1 (unknown space).
  cv::copyMakeBorder(local_mat, total_mat, 
                     pad_south, pad_north, pad_west, pad_east, 
                     cv::BORDER_CONSTANT, cv::Scalar(-1));
  
  // Calculate the new origin of the expanded map.
  double origin_x = local_x - (pad_west * res);
  double origin_y = local_y - (pad_south * res);

  // Buffer for affine transformation.
  cv::Mat warped;
  cv::Point2f src[3], dst[3];

  // Merge other robots' maps into the expanded canvas.
  for (const auto& [id, map] : maps_) {
    if (id == tf_prefix_) continue; // Skip self.
    if (!map) continue;
    if (robots_coords_.find(id) == robots_coords_.end()) continue;

    geometry_msgs::msg::Pose2D pose = robots_coords_[id];
    cv::Mat extra_mat(map->info.height, map->info.width, CV_8SC1, reinterpret_cast<int8_t*>(map->data.data()));

    // Define 3 points (origin, top-right, bottom-left) to compute the affine transform.
    src[0] = {0.f, 0.f};
    src[1] = {static_cast<float>(map->info.width), 0.f};
    src[2] = {0.f, static_cast<float>(map->info.height)};

    float c = std::cos(pose.theta);
    float s = std::sin(pose.theta);
    float ox = map->info.origin.position.x;
    float oy = map->info.origin.position.y;
    float r = map->info.resolution;

    // Transform the 3 source points to the destination pixel coordinates on the global canvas.
    for(int i = 0; i < 3; ++i) {
        float mx = ox + src[i].x * r;
        float my = oy + src[i].y * r;
        // Apply rotation and translation.
        float wx = pose.x + (mx * c - my * s);
        float wy = pose.y + (mx * s + my * c);
        // Convert back to grid indices relative to new origin.
        dst[i].x = (wx - origin_x) / res;
        dst[i].y = (wy - origin_y) / res;
    }

    // Apply the affine warp to align the remote map with the global frame.
    cv::Mat M = cv::getAffineTransform(src, dst);
    cv::warpAffine(extra_mat, warped, M, total_mat.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(-1));
    
    // Merge logic: keep max value (e.g., Obstacle(100) > Free(0) > Unknown(-1)).
    // This ensures obstacles are preserved even if another map sees them as free or unknown.
    cv::max(total_mat, warped, total_mat);
  }

  // Populate the final OccupancyGrid message.
  final_map->header = local_map->header;
  final_map->header.stamp = node_->now();
  final_map->info.resolution = res;
  final_map->info.width = total_mat.cols;
  final_map->info.height = total_mat.rows;
  final_map->info.origin.position.x = origin_x;
  final_map->info.origin.position.y = origin_y;
  final_map->info.origin.orientation.w = 1.0;

  // Copy raw data from the OpenCV matrix to the ROS message vector.
  size_t size = total_mat.total() * total_mat.elemSize();
  final_map->data.resize(size);
  std::memcpy(final_map->data.data(), total_mat.data, size);
}

} // ns multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::MuxMaps>("MuxMaps");
}
#include "easynav_multirobot_exploration/GetExplorationData.hpp"

namespace multirobot_exploration
{

GetExplorationData::GetExplorationData(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_) // Initialize the listener to automatically populate the buffer
{
  // Grab the ROS node and robot prefix from the blackboard to construct the TF frame names later
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** GetExplorationData **");

  // Initialize the global TF node to listen global tf tree
  global_tf_node_ = std::make_shared<rclcpp::Node>(
    "global_tf_node_listener", "/",
    rclcpp::NodeOptions().use_global_arguments(false));
  global_tf_listener_ = std::make_unique<tf2_ros::TransformListener>(global_tf_buffer_, global_tf_node_);

  // Update frontier through topic
  frontier_sub_ = node_->create_subscription<Marker>(
    "frontier_topic", rclcpp::QoS(10).transient_local().reliable(),
    [&](const Marker::SharedPtr marker) {
      last_frontier_ = marker;
    }
  );

  // Update map through topic
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map_topic", rclcpp::QoS(10).transient_local().reliable(),
    [&](const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
      last_map_ = map;
    }
  );
}

BT::NodeStatus
GetExplorationData::tick()
{
  if (!last_frontier_ || !last_map_) {
    RCLCPP_WARN(node_->get_logger(), "Not enough exploration data");
    return BT::NodeStatus::FAILURE;
  }

  try {
    // Get my pose
    std::string map_frame = config().blackboard->get<std::string>("map_frame");
    std::string robot_frame = config().blackboard->get<std::string>("robot_frame");

    Pose pose = getPose(map_frame, robot_frame, tf_buffer_);
    RCLCPP_DEBUG(node_->get_logger(), "Robot at: (%.2f, %.2f)",
                pose.position.x, pose.position.y);

    // Save data in BB
    setOutput("pose", pose);
    setOutput("frontier", last_frontier_->points);
    setOutput("map", *last_map_);
    
    // Get peers poses and save in BB
    std::vector<Pose> peers = getPeersPose();
    if (!peers.empty()) {
      setOutput("peers_pose", peers);
      RCLCPP_DEBUG(node_->get_logger(), "Found %ld peer robots", peers.size());
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "No peer robots found");
    }

    RCLCPP_INFO(node_->get_logger(), "Exploration data saved to blackboard");
    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "TF failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

Pose
GetExplorationData::getPose(
  const std::string & parent_frame,
  const std::string & child_frame,
  tf2::BufferCore& tf_buffer)
{
  Pose pose;
  geometry_msgs::msg::TransformStamped tf_msg;

  // Request the latest available transform from parent to child frame
  // Using TimePointZero avoids timing sync issues by just giving us the most recent TF
  tf_msg = tf_buffer.lookupTransform(parent_frame, child_frame, tf2::TimePointZero);

  // Map the raw TF translation and rotation into a standard Pose message
  pose.position.x = tf_msg.transform.translation.x;
  pose.position.y = tf_msg.transform.translation.y;
  pose.position.z = tf_msg.transform.translation.z;
  pose.orientation = tf_msg.transform.rotation;

  return pose;
}

std::vector<Pose>
GetExplorationData::getPeersPose()
{
  std::vector<Pose> peer_poses;

  // Get the TF frame tree as YAML from the GLOBAL buffer
  std::string tf_yaml = global_tf_buffer_.allFramesAsYAML();
  if (tf_yaml.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Global TF tree is empty");
    return peer_poses;
  }

  // Extract child frames of the global map frame
  std::vector<std::string> peers_robot_frames;
  std::vector<std::string> peers_map_frames = extractChildFrames(tf_yaml, GLOBAL_MAP_FRAME);

  // Recursively extract robot frames from each discovered map frame
  for (const auto& peer_map_frame : peers_map_frames) {
    std::vector<std::string> peers_robot_frame = extractChildFrames(tf_yaml, peer_map_frame);
    peers_robot_frames.insert(peers_robot_frames.end(), peers_robot_frame.begin(), peers_robot_frame.end());
  }
  
  // For each discovered peer robot frame, get the relative pose
  for (const auto& peer_robot_frame : peers_robot_frames) {
    std::string robot_frame = config().blackboard->get<std::string>("robot_frame");

    // Skip if it's the current robot's map frame
    if (peer_robot_frame == robot_frame) {
      continue;
    }

    // Use the global TF buffer to get the pose
    try {    
      Pose peer_pose = getPose(robot_frame, peer_robot_frame, global_tf_buffer_);
      peer_poses.push_back(peer_pose);
      
      RCLCPP_INFO(node_->get_logger(), "Peer robot frame %s at: (%.2f, %.2f)", 
                  peer_robot_frame.c_str(), peer_pose.position.x, peer_pose.position.y);

    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(node_->get_logger(), "Could not get transform for %s: %s", 
                  peer_robot_frame.c_str(), ex.what());
    }
  }
  return peer_poses;
}

std::vector<std::string>
GetExplorationData::extractChildFrames(const std::string & yaml_str, const std::string & parent_frame)
{
  std::vector<std::string> child_frames;
  std::istringstream stream(yaml_str);
  std::string line;
  std::string current_frame;

  // Detects a frame declaration (name that ends with ':' and starts at column 0)
  std::regex frame_regex("^\\s*['\"]?([^'\"\\s:]+)['\"]?:\\s*$");

  // Extracts parent name from "parent: 'name'" or "parent: name" (handles both quote types)
  std::regex parent_regex("^\\s*parent:\\s*['\"]?([^'\"]+)['\"]?\\s*$");

  std::smatch match;

  while (std::getline(stream, line)) {
    // Check if this line is a new frame declaration
    if (std::regex_match(line, match, frame_regex)) {
      current_frame = match[1].str();
    }

    // Check if this line contains the parent field
    else if (std::regex_match(line, match, parent_regex)) {
      if (match[1].str() == parent_frame && !current_frame.empty()) {
        child_frames.push_back(current_frame);
        current_frame = "";
      }
    }
  }

  return child_frames;
}

} // namespace multirobot_exploration


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::GetExplorationData>("GetExplorationData");
}

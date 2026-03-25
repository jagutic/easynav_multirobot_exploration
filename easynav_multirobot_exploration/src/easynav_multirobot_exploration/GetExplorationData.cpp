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

  // Update frontier through topic
  frontier_sub_ = node_->create_subscription<Marker>(
    "frontier_topic", rclcpp::QoS(1).transient_local().reliable(),
    [&](const Marker::SharedPtr marker) {
      std::lock_guard<std::mutex> lock(frontier_mutex_);
      last_frontier_ = marker;
    }
  );

  // Update map through topic
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map_topic", rclcpp::QoS(1).transient_local().reliable(),
    [&](const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
      std::lock_guard<std::mutex> lock(map_mutex_);
      last_map_ = map;
    }
  );
}

BT::NodeStatus
GetExplorationData::tick()
{
  std::scoped_lock<std::mutex, std::mutex> lock(frontier_mutex_, map_mutex_);

  if (!last_frontier_ || !last_map_) {
    RCLCPP_WARN(node_->get_logger(), "Not enough exploration data");
    return BT::NodeStatus::FAILURE;
  }

  try {
    // Move necessary data to BB
    setOutput("pose", getRobotPose());
    setOutput("frontier", last_frontier_->points);
    setOutput("map", *last_map_);

    RCLCPP_INFO(node_->get_logger(), "Exploration data saved");
    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "TF failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

geometry_msgs::msg::Pose
GetExplorationData::getRobotPose()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  geometry_msgs::msg::Pose robot_pose;

  // Request the latest available transform from the global map down to the robot's footprint
  // Using TimePointZero avoids timing sync issues by just giving us the most recent TF
  tf_msg = tf_buffer_.lookupTransform(
    config().blackboard->get<std::string>("map_frame"),
    config().blackboard->get<std::string>("robot_frame"),
    tf2::TimePointZero);

  // Map the raw TF translation and rotation into a standard Pose message
  robot_pose.position.x = tf_msg.transform.translation.x;
  robot_pose.position.y = tf_msg.transform.translation.y;
  robot_pose.position.z = tf_msg.transform.translation.z;
  robot_pose.orientation = tf_msg.transform.rotation;

  RCLCPP_INFO(
    node_->get_logger(),
    "Pose obtained: x= %.2f, y=%.2f, Y=%.2f",
    robot_pose.position.x, robot_pose.position.y,
    tf2::getYaw(robot_pose.orientation)
  );

  return robot_pose;
}


} // namespace multirobot_exploration


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::GetExplorationData>("GetExplorationData");
}

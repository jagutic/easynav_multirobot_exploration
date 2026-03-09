#include "easynav_multirobot_exploration/GetPose.hpp"

namespace multirobot_exploration
{

GetPose::GetPose(
    const std::string& name,
    const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_) // Initialize the listener to automatically populate the buffer
{
  // Grab the ROS node and robot prefix from the blackboard to construct the TF frame names later
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** GetPose **");
}

BT::NodeStatus
GetPose::tick()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  geometry_msgs::msg::Pose pose;

  try {
    // Request the latest available transform from the global map down to the robot's footprint
    // Using TimePointZero avoids timing sync issues by just giving us the most recent TF
    tf_msg = tf_buffer_.lookupTransform(
      config().blackboard->get<std::string>("map_frame"),
      config().blackboard->get<std::string>("robot_frame"),
      tf2::TimePointZero);

    // Map the raw TF translation and rotation into a standard Pose message
    pose.position.x = tf_msg.transform.translation.x;
    pose.position.y = tf_msg.transform.translation.y;
    pose.position.z = tf_msg.transform.translation.z;
    pose.orientation = tf_msg.transform.rotation;
    
    // Push the pose to the blackboard so navigation nodes can access it
    setOutput("pose", pose);

    // Convert the quaternion to yaw just to make the console log human-readable
    RCLCPP_INFO(
      node_->get_logger(),
      "Pose obtained: x= %.2f, y=%.2f, Y=%.2f",
      pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)
    );
    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException & ex) {
    // Gracefully catch missing frames or delayed trees instead of crashing the whole BT
    RCLCPP_WARN(node_->get_logger(), "TF failed: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }
}

} // namespace multirobot_exploration


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::GetPose>("GetPose");
}
#include "easynav_multirobot_exploration/GetPose.hpp"

namespace multirobot_exploration
{

GetPose::GetPose(
    const std::string& name,
    const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_prefix_ = config().blackboard->get<std::string>("tf_prefix");

  RCLCPP_INFO(node_->get_logger(), "** GetPose **");
}

BT::NodeStatus
GetPose::tick()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  geometry_msgs::msg::Pose pose;

  try {
    tf_msg = tf_buffer_.lookupTransform(
      tf_prefix_ + "/map", tf_prefix_ + "/base_footprint", tf2::TimePointZero);

    pose.position.x = tf_msg.transform.translation.x;
    pose.position.y = tf_msg.transform.translation.y;
    pose.position.z = tf_msg.transform.translation.z;
    pose.orientation = tf_msg.transform.rotation;
    setOutput("pose", pose);

    RCLCPP_INFO(
      node_->get_logger(),
      "Pose obtained: x= %.2f, y=%.2f, Y=%.2f",
      pose.position.x, pose.position.y, tf2::getYaw(pose.orientation)
    );
    return BT::NodeStatus::SUCCESS;

  } catch (const tf2::TransformException & ex) {
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
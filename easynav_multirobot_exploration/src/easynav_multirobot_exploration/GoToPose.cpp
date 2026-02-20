#include "easynav_multirobot_exploration/GoToPose.hpp"

namespace multirobot_exploration
{

GoToPose::GoToPose(const std::string& name, const BT::NodeConfig& conf)
  : BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** GoToPose **");

  nav_client_ = std::make_shared<easynav::GoalManagerClient>(node_);

}

BT::NodeStatus
GoToPose::tick()
{
  return BT::NodeStatus::SUCCESS;
}

void
GoToPose::halt()
{
  nav_client_->cancel();
  RCLCPP_INFO(node_->get_logger(), "GoToPose halted.");
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::GoToPose>("GoToPose");
}
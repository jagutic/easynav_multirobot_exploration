#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"

namespace multirobot_exploration
{

ChooseFrontierGoal::ChooseFrontierGoal(const std::string& name, const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** ChooseFrontierGoal **");
}

BT::NodeStatus ChooseFrontierGoal::tick()
{
  return BT::NodeStatus::SUCCESS;
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
}
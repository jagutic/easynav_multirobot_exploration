#include "easynav_multirobot_exploration/IsExplored.hpp"

namespace multirobot_exploration
{

IsExplored::IsExplored(
  const std::string& name,
  const BT::NodeConfig& conf)
  : BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** IsExplored **");
}

BT::NodeStatus IsExplored::tick()
{
  return BT::NodeStatus::FAILURE;
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::IsExplored>("IsExplored");
}
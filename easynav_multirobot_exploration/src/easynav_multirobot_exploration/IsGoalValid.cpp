#include "easynav_multirobot_exploration/IsGoalValid.hpp"

namespace multirobot_exploration
{

IsGoalValid::IsGoalValid(const std::string & name, const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** IsGoalValid **");
}

BT::NodeStatus IsGoalValid::tick()
{
  return BT::NodeStatus::SUCCESS;
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::IsGoalValid>("IsGoalValid");
}

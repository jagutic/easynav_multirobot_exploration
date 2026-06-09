#include "easynav_multirobot_exploration/IsExplored.hpp"

namespace multirobot_exploration
{

IsExplored::IsExplored(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** IsExplored **");
}

BT::NodeStatus IsExplored::tick()
{
  // Get BB frontier
  std::vector<Point> frontier_to_check;
  BT::Result result = getInput("frontier_to_check", frontier_to_check);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No frontier");
    return BT::NodeStatus::FAILURE;
  }


  // Only consider finished when there are not more frontiers to explore
  if (frontier_to_check.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No more frontiers to explore !!!");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::IsExplored>("IsExplored");
}

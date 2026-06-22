#include "easynav_multirobot_exploration/ProcessGoal.hpp"

namespace easynav_multirobot_exploration
{

ProcessGoal::ProcessGoal(const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** ProcessGoal **");

  node_->declare_parameter("min_cost_diff", 0.1);
  node_->get_parameter("min_cost_diff", min_cost_diff_);

  RCLCPP_INFO(node_->get_logger(), "Minimum cost difference to change goal: %.2f", min_cost_diff_);
}

BT::NodeStatus ProcessGoal::tick()
{
  // Get BB goal with cost
  PoseWithCost goal_with_cost;
  BT::Result result = getInput("goal_with_cost", goal_with_cost);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No goal with cost");
    return BT::NodeStatus::FAILURE;
  }

  // Avoid constant change in goal
  if (last_cost_ && std::abs(*last_cost_ - goal_with_cost.cost) < min_cost_diff_) {

    // Keep current goal if the new one is not much better
    RCLCPP_INFO(
      node_->get_logger(),
      "Not changing goal, cost difference: %.2f",
      std::abs(*last_cost_ - goal_with_cost.cost)
    );
    return BT::NodeStatus::SUCCESS;
  }

  // Save new cost and output goal
  if (!last_cost_) {
    last_cost_ = new double();
  }
  *last_cost_ = goal_with_cost.cost;

  setOutput("goal", goal_with_cost.pose);
  RCLCPP_INFO(node_->get_logger(), "New goal selected with cost: %.2f", goal_with_cost.cost);
  return BT::NodeStatus::SUCCESS;
}

} // namespace easynav_multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<easynav_multirobot_exploration::ProcessGoal>("ProcessGoal");
}

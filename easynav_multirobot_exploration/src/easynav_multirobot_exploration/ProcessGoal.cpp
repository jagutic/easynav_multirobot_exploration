#include "easynav_multirobot_exploration/ProcessGoal.hpp"
#include <algorithm>
#include <cmath>
#include <ranges>

namespace easynav_multirobot_exploration {

ProcessGoal::ProcessGoal(const std::string &name, const BT::NodeConfig &conf)
    : BT::SyncActionNode(name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  node_->declare_parameter("proximity_threshold", 0.5);
  node_->declare_parameter("improvement_ratio", 0.5);

  node_->get_parameter("proximity_threshold", proximity_threshold_);
  node_->get_parameter("improvement_ratio", improvement_ratio_);

  RCLCPP_INFO(node_->get_logger(), "** ProcessGoal **");
  RCLCPP_INFO(node_->get_logger(), "Proximity threshold: %.2f m", proximity_threshold_);
  RCLCPP_INFO(node_->get_logger(), "Improvement ratio: %.2f", improvement_ratio_);
}

BT::NodeStatus
ProcessGoal::tick()
{
  PoseWithCost candidate;
  if (!getInput("goal_with_cost", candidate).has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No goal_with_cost input");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<Point> goals_list;
  if (!getInput("goals_list", goals_list).has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No goals_list input");
    return BT::NodeStatus::FAILURE;
  }

  // Switch or keep goal based on proximity and cost
  Pose final_goal = manageCandidate(candidate, goals_list);

  setOutput("goal", final_goal);
  return BT::NodeStatus::SUCCESS;
}

bool
ProcessGoal::stillAlive(const Pose &pose, const std::vector<Point> &poses)
{
  return std::any_of(
    poses.begin(), poses.end(),
    [&pose, this](const Point &p) {
      double dx = pose.position.x - p.x;
      double dy = pose.position.y - p.y;
      return (dx * dx + dy * dy) < (proximity_threshold_ * proximity_threshold_);
    });
}

double
ProcessGoal::getImprovedCost(const double cost)
{
  double improvement;
  if (cost < 0) {
    // For negative costs, more negative is better.
    // e.g., current=-10, ratio=0.25 -> threshold=-13.33. New cost must be < -13.33 to switch.
    improvement = cost / (1 - improvement_ratio_);
  } else {
    // For positive costs, smaller is better.
    // e.g., current=10, ratio=0.25 -> threshold=7.5. New cost must be < 7.5 to switch.
    improvement = cost * (1 - improvement_ratio_);
  }

  return improvement;
}

Pose
ProcessGoal::manageCandidate(const PoseWithCost &candidate, const std::vector<Point> &goals_list)
{
  PoseWithCost goal = candidate;

  if (last_goal_.has_value()) {
    double improved_cost = getImprovedCost(last_goal_->cost);
    bool still_alive = stillAlive(last_goal_->pose, goals_list);

    // Keep current goal
    if (still_alive && candidate.cost >= improved_cost) {
      RCLCPP_INFO(node_->get_logger(), "Keeping goal: (cost %.2f)",
                  last_goal_->cost);
      goal = *last_goal_;

      // Switch because goal disappeared
    } else if (still_alive) {
      RCLCPP_INFO(node_->get_logger(),
                  "Switching goal: new cost (%.2f) better than old cost (%.2f)",
                  candidate.cost, last_goal_->cost);

      // Switch because candidate is better
    } else {
      RCLCPP_INFO(node_->get_logger(), "Switching goal: current disappeared");
    }
  }

  last_goal_ = goal;
  return goal.pose;
}

} // namespace easynav_multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<easynav_multirobot_exploration::ProcessGoal>(
      "ProcessGoal");
}

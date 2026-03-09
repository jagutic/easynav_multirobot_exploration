#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"

namespace multirobot_exploration
{

ChooseFrontierGoal::ChooseFrontierGoal(
  const std::string& name,
  const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** ChooseFrontierGoal **");

  // Update frontier through topic
  frontier_sub_ = node_->create_subscription<Marker>(
    "frontier_topic", rclcpp::QoS(1).transient_local().reliable(),
    [&](const Marker::SharedPtr marker) {
      frontier_ = marker->points;
    });
}

BT::NodeStatus
ChooseFrontierGoal::tick()
{
  Pose robot_pose;
  BT::Result result = getInput("robot_pose", robot_pose);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No pose");
    return BT::NodeStatus::FAILURE;
  }

  if (frontier_.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "No frontier yet");
    return BT::NodeStatus::FAILURE;
  }

  Pose frontier_goal = calc_closest_goal(robot_pose);
  setOutput("frontier_goal", frontier_goal);

  RCLCPP_INFO(node_->get_logger(), "Frontier goal selected");
  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::Pose
ChooseFrontierGoal::calc_closest_goal(const Pose & current_pose)
{
  geometry_msgs::msg::Pose frontier_goal;
  double min_dist_sq = std::numeric_limits<double>::max();

  for (const auto & frontier_point : frontier_) {
    double dx = frontier_point.x - current_pose.position.x;
    double dy = frontier_point.y - current_pose.position.y;
    double dist_sq = dx * dx + dy * dy;

    // Squared distances for efficiency
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      frontier_goal.position.x = frontier_point.x;
      frontier_goal.position.y = frontier_point.y;

      // Orientate goal as vector robot->goal
      double yaw = std::atan2(dy, dx);

      frontier_goal.orientation.z = std::sin(yaw / 2.0);
      frontier_goal.orientation.w = std::cos(yaw / 2.0);
    }
  }

  return frontier_goal;
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
}
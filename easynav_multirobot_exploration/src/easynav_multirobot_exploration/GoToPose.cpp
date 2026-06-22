#include "easynav_multirobot_exploration/GoToPose.hpp"

namespace easynav_multirobot_exploration
{

GoToPose::GoToPose(const std::string & name, const BT::NodeConfig & conf)
: BT::ActionNodeBase(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** GoToPose **");

  nav_client_ = std::make_shared<easynav::GoalManagerClient>(node_);
}

BT::NodeStatus
GoToPose::tick()
{
  // Manage response
  easynav_interfaces::msg::NavigationControl response;

  switch (nav_client_->get_state()) {
    // No action in these states
    case easynav::GoalManagerClient::State::SENT_GOAL:
      RCLCPP_INFO(node_->get_logger(), "GOAL SENT");
      return BT::NodeStatus::RUNNING;
    case easynav::GoalManagerClient::State::SENT_PREEMPT:
      RCLCPP_INFO(node_->get_logger(), "PREEMPT SENT");
      return BT::NodeStatus::RUNNING;

    // Manage normal state
    case easynav::GoalManagerClient::State::IDLE:
      RCLCPP_INFO(node_->get_logger(), "IDLE");

      // Send goal
      if (!send_goal()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal");
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;

    // Manage navigating state
    case easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING:
      RCLCPP_INFO(node_->get_logger(), "NAVIGATING");

      // Send goal
      if (!send_goal()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to send goal, ACCEPTED_AND_NAVIGATING");
        return BT::NodeStatus::FAILURE;
      }

      // Get feedback
      response = nav_client_->get_feedback();
      RCLCPP_INFO(
        node_->get_logger(),
        "FEEDBACK: Distance to goal: %.2f, Navigation time: %d sec",
        response.distance_to_goal, response.navigation_time.sec
      );
      return BT::NodeStatus::RUNNING;

    // Manage successful navigation
    case easynav::GoalManagerClient::State::NAVIGATION_FINISHED:
      response = nav_client_->get_result();
      RCLCPP_INFO(node_->get_logger(), "NAVIGATION FINISHED: %s", response.status_message.c_str());

      nav_client_->reset();
      return BT::NodeStatus::SUCCESS;

    // Manage failed navigation
    case easynav::GoalManagerClient::State::NAVIGATION_CANCELLED:
    case easynav::GoalManagerClient::State::NAVIGATION_FAILED:
      response = nav_client_->get_result();
      RCLCPP_WARN(node_->get_logger(), "NAVIGATION FAILED: %s", response.status_message.c_str());

      nav_client_->reset();
      return BT::NodeStatus::FAILURE;

    // Manage error
    case easynav::GoalManagerClient::State::ERROR:
      RCLCPP_ERROR(node_->get_logger(), "GoalManagerClient ERROR");
      nav_client_->reset();
      return BT::NodeStatus::FAILURE;

    // Manage undefined
    default:
      RCLCPP_ERROR(node_->get_logger(), "GoalManagerClient UNDEFINED STATE");
      nav_client_->reset();
      return BT::NodeStatus::FAILURE;
  }
}

void
GoToPose::halt()
{
  nav_client_->cancel();
  RCLCPP_INFO(node_->get_logger(), "GoToPose halted.");
}

bool
GoToPose::send_goal()
{
  // Get goal from BB
  Pose goal_pose;
  BT::Result result = getInput("goal_pose", goal_pose);

  if (!result.has_value()) { // No goal -> failure
    RCLCPP_ERROR(node_->get_logger(), "No goal pose");
    return false;
  }

  // Same goal, not resending
  if (last_goal_pose_ && *last_goal_pose_ == goal_pose) {
    return true;
  }

  // Send goal with easynav client
  PoseStamped goal;
  goal.header.frame_id = config().blackboard->get<std::string>("map_frame");
  goal.header.stamp = node_->now();
  goal.pose = goal_pose;

  RCLCPP_DEBUG(node_->get_logger(), "Sending goal to (%.2f, %.2f)", goal.pose.position.x,
      goal.pose.position.y);
  nav_client_->send_goal(goal);

  // Update last goal pose
  if (!last_goal_pose_) {
    last_goal_pose_ = new Pose();
  }
  *last_goal_pose_ = goal_pose;
  return true;
}

} // namespace easynav_multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<easynav_multirobot_exploration::GoToPose>("GoToPose");
}

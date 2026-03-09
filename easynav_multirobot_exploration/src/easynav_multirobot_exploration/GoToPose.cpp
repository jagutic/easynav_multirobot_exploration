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
  easynav_interfaces::msg::NavigationControl response;
  easynav::GoalManagerClient::State state;
  state = nav_client_->get_state();
  
  switch (state) {
    case easynav::GoalManagerClient::State::SENT_GOAL:
    case easynav::GoalManagerClient::State::SENT_PREEMPT:
      return BT::NodeStatus::RUNNING;
    
    // Manage error state
    case easynav::GoalManagerClient::State::ERROR:
      RCLCPP_ERROR(node_->get_logger(), "Error with goal manager client");
      nav_client_->reset();
      return BT::NodeStatus::FAILURE;

    // Manage finished state
    case easynav::GoalManagerClient::State::NAVIGATION_CANCELLED:
    case easynav::GoalManagerClient::State::NAVIGATION_FAILED:
      response = nav_client_->get_result();
      RCLCPP_WARN(node_->get_logger(), "%s", response.status_message.c_str());

      nav_client_->reset();
      return BT::NodeStatus::FAILURE;

    case easynav::GoalManagerClient::State::NAVIGATION_FINISHED:
      response = nav_client_->get_result();
      RCLCPP_INFO(node_->get_logger(), "%s", response.status_message.c_str());

      nav_client_->reset();
      return BT::NodeStatus::SUCCESS;
  
    // Manage navigating state
    case easynav::GoalManagerClient::State::ACCEPTED_AND_NAVIGATING:
      response = nav_client_->get_feedback();
      RCLCPP_INFO(
        node_->get_logger(),
        "Distance to goal: %.2f",
        response.distance_to_goal
      );
      RCLCPP_INFO(
        node_->get_logger(),
        "Navigation time: %d sec",
        response.navigation_time.sec
      );

      // if (response.distance_to_goal < GOAL_RADIUS) {
      //   nav_client_->cancel();
      //   return BT::NodeStatus::SUCCESS;
      // }
      return BT::NodeStatus::RUNNING;

    // Manage normal state, send goal to begin navigation
    case easynav::GoalManagerClient::State::IDLE:
    {
      Pose goal_pose;
      BT::Result result = getInput("goal_pose", goal_pose);

      if (!result.has_value()) { // No goal -> failure
        RCLCPP_ERROR(node_->get_logger(), "No goal pose");
        return BT::NodeStatus::FAILURE;
      }

      // Fill goal msg
      PoseStamped goal;
      goal.header.frame_id = config().blackboard->get<std::string>("map_frame");
      goal.header.stamp = node_->now();
      goal.pose = goal_pose;
      nav_client_->send_goal(goal);
  
      RCLCPP_INFO(node_->get_logger(), "Goal sent");
      return BT::NodeStatus::RUNNING;
    }

    default:
      RCLCPP_ERROR(node_->get_logger(), "Undefined goal manager client state");
      nav_client_->reset();
      return BT::NodeStatus::FAILURE;
  }
}

void
GoToPose::halt()
{
  nav_client_->cancel();
  nav_client_->reset();
  RCLCPP_INFO(node_->get_logger(), "GoToPose halted.");
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::GoToPose>("GoToPose");
}
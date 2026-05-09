#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"

namespace multirobot_exploration
{

ChooseFrontierGoal::ChooseFrontierGoal(
  const std::string & name,
  const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** ChooseFrontierGoal **");

  node_->declare_parameter("policy", POLICY_NEAREST_FRONTIER);
  node_->declare_parameter("distance_weight", 1.0);
  node_->declare_parameter("separation_weight", 1.0);

  node_->get_parameter("policy", policy_);
  node_->get_parameter("distance_weight", distance_weight_);
  node_->get_parameter("separation_weight", separation_weight_);

  switch (policy_) {
  case POLICY_NEAREST_FRONTIER:
    RCLCPP_INFO(node_->get_logger(), "Using policy: NEAREST_FRONTIER");
    break;
  case POLICY_BETTER_FRONTIER:
    RCLCPP_INFO(node_->get_logger(), "Using policy: BETTER_FRONTIER");
    break;
  default:
    RCLCPP_WARN(node_->get_logger(), "Unknown policy, defaulting to NEAREST_FRONTIER");
  }
}

BT::NodeStatus
ChooseFrontierGoal::tick()
{
  // Get BB pose
  Pose robot_pose;
  BT::Result result = getInput("robot_pose", robot_pose);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No pose");
    return BT::NodeStatus::FAILURE;
  }

  // Get BB frontier
  std::vector<Point> robot_frontier;
  result = getInput("robot_frontier", robot_frontier);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No frontier");
    return BT::NodeStatus::FAILURE;
  }

  if (robot_frontier.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Frontier empty, no possible goal selection");
    return BT::NodeStatus::FAILURE;
  }

  // Decide goal out of the actual frontier
  Pose frontier_goal;
  std::vector<Pose> peers_robot_pose;

  switch (policy_) {
  // Choose closest frontier point to robot
  case POLICY_NEAREST_FRONTIER:
    frontier_goal = calc_closest_goal(robot_pose, robot_frontier);
    break;
  
  // Choose best frontier point according to cost function (distance to robot and distance to peers)
  case POLICY_BETTER_FRONTIER:
    getInput("peers_robot_pose", peers_robot_pose);
    frontier_goal = calc_best_goal(robot_pose, peers_robot_pose, robot_frontier);
    break;

  // Undefined state
  default:
    RCLCPP_WARN(node_->get_logger(), "Policy undefined");
  }

  setOutput("frontier_goal", frontier_goal);
  RCLCPP_INFO(node_->get_logger(), "Frontier goal selected");

  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::Pose
ChooseFrontierGoal::calc_closest_goal(
  const Pose & pose, const std::vector<Point> & frontier)
{
  int best_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  // Select closest point out of all points in frontier
  for (int i = 0; i < frontier.size(); ++i) {
    double dx = frontier[i].x - pose.position.x;
    double dy = frontier[i].y - pose.position.y;
    double dist_sq = dx * dx + dy * dy;

    // Squared distances for efficiency
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_idx = i;
    }
  }

  // Fill goal with best frontier point
  geometry_msgs::msg::Pose frontier_goal;
  frontier_goal.position.x = frontier[best_idx].x;
  frontier_goal.position.y = frontier[best_idx].y;
  
  // Orientate goal as vector robot->goal
  double yaw = std::atan2(
    frontier[best_idx].y - pose.position.y, frontier[best_idx].x - pose.position.x);

  frontier_goal.orientation.z = std::sin(yaw / 2.0);
  frontier_goal.orientation.w = std::cos(yaw / 2.0);

  return frontier_goal;
}

geometry_msgs::msg::Pose
ChooseFrontierGoal::calc_best_goal(
  const geometry_msgs::msg::Pose & pose, 
  const std::vector<geometry_msgs::msg::Pose> & peers_pose,
  const std::vector<geometry_msgs::msg::Point> & frontier)
{
  int best_idx = 0;
  double min_cost = std::numeric_limits<double>::max();

  // Select best point out of all points in frontier
  for (int i = 0; i < frontier.size(); ++i) {

    // Distance to my position
    double dx_dist = frontier[i].x - pose.position.x;
    double dy_dist = frontier[i].y - pose.position.y;
    double dist_to_robot = std::hypot(dx_dist, dy_dist);

    // Sum of distance to peers
    double dist_to_peer_sum = 0.0;

    if (!peers_pose.empty()) {
      for (const auto & peer : peers_pose) {
        double dist_to_peer = std::hypot(
          frontier[i].x - peer.position.x, frontier[i].y - peer.position.y);

        dist_to_peer_sum += dist_to_peer;
      }
    }

    // Minimize distance to robot and maximize distance to peers
    double cost = (distance_weight_ * dist_to_robot) - (separation_weight_ * dist_to_peer_sum);
    if (cost < min_cost) {
      min_cost = cost;
      best_idx = i;
    }
  }

  // Fill goal with best frontier point
  geometry_msgs::msg::Pose frontier_goal;
  frontier_goal.position.x = frontier[best_idx].x;
  frontier_goal.position.y = frontier[best_idx].y;
  
  // Orientate goal as vector robot->goal
  double yaw = std::atan2(
    frontier[best_idx].y - pose.position.y, frontier[best_idx].x - pose.position.x);

  frontier_goal.orientation.z = std::sin(yaw / 2.0);
  frontier_goal.orientation.w = std::cos(yaw / 2.0);

  return frontier_goal;
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
}

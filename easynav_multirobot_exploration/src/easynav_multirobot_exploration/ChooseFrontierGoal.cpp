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
  node_->declare_parameter("min_cost_diff", 0.1);

  node_->get_parameter("policy", policy_);
  node_->get_parameter("distance_weight", distance_weight_);
  node_->get_parameter("separation_weight", separation_weight_);
  node_->get_parameter("min_cost_diff", min_cost_diff_);

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

  RCLCPP_INFO(node_->get_logger(), "Distance weight: %.2f", distance_weight_);
  RCLCPP_INFO(node_->get_logger(), "Separation weight: %.2f", separation_weight_);
  RCLCPP_INFO(node_->get_logger(), "Minimum cost difference to change goal: %.2f", min_cost_diff_);
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
    RCLCPP_ERROR(node_->get_logger(), "Frontier empty");
    return BT::NodeStatus::FAILURE;
  }

  // Decide goal out of the actual frontier
  double goal_cost;
  Pose frontier_goal;
  std::vector<Pose> peers_robot_pose;

  switch (policy_) {
  // Choose closest frontier point to robot
    case POLICY_NEAREST_FRONTIER:
      std::tie(frontier_goal, goal_cost) = calc_closest_goal(robot_pose, robot_frontier);
      break;

  // Choose best frontier point according to cost function (distance to robot and distance to peers)
    case POLICY_BETTER_FRONTIER:
      getInput("peers_robot_pose", peers_robot_pose);
      std::tie(frontier_goal, goal_cost) = calc_best_goal(robot_pose, peers_robot_pose,
        robot_frontier);
      break;

  // Undefined state
    default:
      RCLCPP_ERROR(node_->get_logger(), "Policy undefined");
      return BT::NodeStatus::FAILURE;
  }

  // Avoid constant change in goal
  double last_goal_cost;
  bool last_goal_cost_exists = config().blackboard->get("last_goal_cost", last_goal_cost);

  if (last_goal_cost_exists) {
    if (std::abs(last_goal_cost - goal_cost) < min_cost_diff_) {
      // Keep current goal if the new one is not much better
      RCLCPP_INFO(
        node_->get_logger(),
        "Not changing goal, cost difference: %.2f",
        std::abs(last_goal_cost - goal_cost)
      );
      return BT::NodeStatus::SUCCESS;
    }

  } else {
    RCLCPP_INFO(node_->get_logger(), "No last goal cost yet");
  }

  // Set or change goal
  setOutput("frontier_goal", frontier_goal);
  setOutput("frontier_goal_cost", goal_cost);
  RCLCPP_INFO(node_->get_logger(), "New frontier goal selected, with cost: %.2f", goal_cost);

  return BT::NodeStatus::SUCCESS;
}

std::pair<geometry_msgs::msg::Pose, double>
ChooseFrontierGoal::calc_closest_goal(
  const Pose & pose, const std::vector<Point> & frontier)
{
  int best_idx = 0;
  double min_dist_sq = std::numeric_limits<double>::max();

  // Select closest point out of all points in frontier
  for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {
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

  double cost = std::sqrt(min_dist_sq);
  return std::make_pair(frontier_goal, cost);
}

std::pair<geometry_msgs::msg::Pose, double>
ChooseFrontierGoal::calc_best_goal(
  const geometry_msgs::msg::Pose & pose,
  const std::vector<geometry_msgs::msg::Pose> & peers_pose,
  const std::vector<geometry_msgs::msg::Point> & frontier)
{
  int best_idx = 0;
  double min_cost = std::numeric_limits<double>::max();

  // Select best point out of all points in frontier
  for (int i = 0; i < static_cast<int>(frontier.size()); ++i) {

    // Distance to my position
    double dx_dist = frontier[i].x - pose.position.x;
    double dy_dist = frontier[i].y - pose.position.y;
    double dist_to_robot_sq = dx_dist * dx_dist + dy_dist * dy_dist;

    // Find the minimum distance to any peer
    double min_dist_to_peer_sq = std::numeric_limits<double>::max();

    if (!peers_pose.empty()) {
      for (const auto & peer : peers_pose) {
        double dist_to_peer_sq =
          (frontier[i].x - peer.position.x) * (frontier[i].x - peer.position.x) +
          (frontier[i].y - peer.position.y) * (frontier[i].y - peer.position.y);

        if (dist_to_peer_sq < min_dist_to_peer_sq) {
          min_dist_to_peer_sq = dist_to_peer_sq;
        }
      }
    } else {
      // If no peers, treat the closest peer distance as 0 so it doesn't affect the cost
      min_dist_to_peer_sq = 0.0;
    }

    double true_dist_robot = std::sqrt(dist_to_robot_sq);
    double true_dist_peer = std::sqrt(min_dist_to_peer_sq);

    // Minimize distance to robot and maximize distance to the CLOSEST peer
    double cost = (distance_weight_ * true_dist_robot) - (separation_weight_ * true_dist_peer);
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

  return std::make_pair(frontier_goal, min_cost);
}

} // namespace multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
}

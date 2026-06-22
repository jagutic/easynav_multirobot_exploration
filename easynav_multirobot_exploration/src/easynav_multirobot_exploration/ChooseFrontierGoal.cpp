#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"

namespace easynav_multirobot_exploration
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

  RCLCPP_INFO(node_->get_logger(), "Distance weight: %.2f", distance_weight_);
  RCLCPP_INFO(node_->get_logger(), "Separation weight: %.2f", separation_weight_);
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
  PoseWithCost frontier_goal;
  std::vector<Pose> peers_robot_pose;

  switch (policy_) {
  // Choose closest frontier point to robot
    case POLICY_NEAREST_FRONTIER:
      frontier_goal = calc_closest_goal(robot_pose, robot_frontier);
      break;

  // Choose best frontier point according to cost function (distance to robot and distance to peers)
    case POLICY_BETTER_FRONTIER:
      getInput("peers_robot_pose", peers_robot_pose);
      frontier_goal = calc_best_goal(robot_pose, peers_robot_pose,
        robot_frontier);
      break;

  // Undefined state
    default:
      RCLCPP_ERROR(node_->get_logger(), "Policy undefined");
      return BT::NodeStatus::FAILURE;
  }

  // Set goal
  setOutput("frontier_goal", frontier_goal);
  return BT::NodeStatus::SUCCESS;
}

PoseWithCost
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
  PoseWithCost frontier_goal;
  frontier_goal.pose.position.x = frontier[best_idx].x;
  frontier_goal.pose.position.y = frontier[best_idx].y;

  // Orientate goal as vector robot->goal
  double yaw = std::atan2(
    frontier[best_idx].y - pose.position.y, frontier[best_idx].x - pose.position.x);

  frontier_goal.pose.orientation.z = std::sin(yaw / 2.0);
  frontier_goal.pose.orientation.w = std::cos(yaw / 2.0);

  frontier_goal.cost = std::sqrt(min_dist_sq);
  return frontier_goal;
}

PoseWithCost
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
  PoseWithCost frontier_goal;
  frontier_goal.pose.position.x = frontier[best_idx].x;
  frontier_goal.pose.position.y = frontier[best_idx].y;

  // Orientate goal as vector robot->goal
  double yaw = std::atan2(
    frontier[best_idx].y - pose.position.y, frontier[best_idx].x - pose.position.x);

  frontier_goal.pose.orientation.z = std::sin(yaw / 2.0);
  frontier_goal.pose.orientation.w = std::cos(yaw / 2.0);

  frontier_goal.cost = min_cost;
  return frontier_goal;
}

} // namespace easynav_multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<easynav_multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
}

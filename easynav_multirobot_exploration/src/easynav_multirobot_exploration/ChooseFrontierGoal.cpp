#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"

namespace easynav_multirobot_exploration {

ChooseFrontierGoal::ChooseFrontierGoal(const std::string &name,
                                       const BT::NodeConfig &conf)
    : BT::SyncActionNode(name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** ChooseFrontierGoal **");

  // Get policy metrics and weights
  node_->declare_parameter("proximity_policy", EUCLIDEAN_PROXIMITY_POLICY);
  node_->declare_parameter("separation_policy", NO_SEPARATION_POLICY);
  node_->declare_parameter("proximity_weight", 1.0);
  node_->declare_parameter("separation_weight", 1.0);

  node_->get_parameter("proximity_policy", proximity_policy_);
  node_->get_parameter("separation_policy", separation_policy_);
  node_->get_parameter("proximity_weight", proximity_weight_);
  node_->get_parameter("separation_weight", separation_weight_);

  // Inform about proximity and separation policies
  switch (proximity_policy_) {
  case EUCLIDEAN_PROXIMITY_POLICY:
    RCLCPP_INFO(node_->get_logger(), "Using: EUCLIDEAN_PROXIMITY_POLICY");
    break;
  case REAL_PROXIMITY_POLICY:
    RCLCPP_INFO(node_->get_logger(), "Using: REAL_PROXIMITY_POLICY");
    break;
  default:
    RCLCPP_WARN(node_->get_logger(),
                "Unknown policy, defaulting to EUCLIDEAN_PROXIMITY_POLICY");
    proximity_policy_ = EUCLIDEAN_PROXIMITY_POLICY;
  }

  switch (separation_policy_) {
  case NO_SEPARATION_POLICY:
    RCLCPP_INFO(node_->get_logger(), "Using: NO_SEPARATION_POLICY");
    break;
  case POSITION_SEPARATION_POLICY:
    RCLCPP_INFO(node_->get_logger(), "Using: POSITION_SEPARATION_POLICY");
    break;
  case GOAL_SEPARATION_POLICY:
    RCLCPP_INFO(node_->get_logger(), "Using: GOAL_SEPARATION_POLICY");
    break;
  default:
    RCLCPP_WARN(node_->get_logger(),
                "Unknown policy, defaulting to NO_SEPARATION_POLICY");
    separation_policy_ = NO_SEPARATION_POLICY;
  }

  // Inform about weights
  RCLCPP_INFO(node_->get_logger(), "Proximity weight: %.2f", proximity_weight_);
  RCLCPP_INFO(node_->get_logger(), "Separation weight: %.2f",
              separation_weight_);
}

BT::NodeStatus ChooseFrontierGoal::tick() {
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
    RCLCPP_INFO(node_->get_logger(),
                "Frontier empty, not choosing frontier goal");
    return BT::NodeStatus::FAILURE;
  }

  // Get BB peers depending on separation policy selected
  std::vector<Pose> robot_peers;
  
  switch (separation_policy_) {
  case NO_SEPARATION_POLICY:
    break;
  
  case POSITION_SEPARATION_POLICY:
    result = getInput("peers_robot_pose", robot_peers);
      if (!result.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "No peers pose input");
        return BT::NodeStatus::FAILURE;
      }
    break;

  case GOAL_SEPARATION_POLICY:
    result = getInput("peers_robot_goal", robot_peers);
      if (!result.has_value()) {
        RCLCPP_ERROR(node_->get_logger(), "No peers goal input");
        return BT::NodeStatus::FAILURE;
      }
    break;

  default:
    RCLCPP_ERROR(node_->get_logger(),
                 "Wrong separation policy, not choosing frontier goal");
    return BT::NodeStatus::FAILURE;
  }

  if (robot_peers.empty()) {
    RCLCPP_INFO(node_->get_logger(), "No peers for separation");
  }

  // Choose and set the best frontier goal
  auto frontier_goal = get_best_goal(robot_pose, robot_frontier, robot_peers);
  setOutput("frontier_goal", frontier_goal);

  RCLCPP_INFO(node_->get_logger(), "Best frontier candidate, with cost: %.2f",
              frontier_goal.cost);
  return BT::NodeStatus::SUCCESS;
}

std::function<double(const geometry_msgs::msg::Point &)> 
ChooseFrontierGoal::get_proximity_calculator(const geometry_msgs::msg::Pose & pose)
{
  switch (proximity_policy_) {
    case EUCLIDEAN_PROXIMITY_POLICY:
      return [&pose](const geometry_msgs::msg::Point & pt) -> double {
        return std::hypot(pt.x - pose.position.x, pt.y - pose.position.y);
      };

    case REAL_PROXIMITY_POLICY:
      return [this, &pose](const geometry_msgs::msg::Point & pt) -> double {
        // TODO
        return 0.0;
      };

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown proximity policy");
      return [](const geometry_msgs::msg::Point &) { return 0.0; };
  }
}

std::function<double(const geometry_msgs::msg::Point &)> 
ChooseFrontierGoal::get_separation_calculator(const std::vector<Pose> & peers)
{
  switch (separation_policy_) {
    case NO_SEPARATION_POLICY:
      return [](const geometry_msgs::msg::Point &) -> double {
        return 0.0;
      };

    case POSITION_SEPARATION_POLICY:
    case GOAL_SEPARATION_POLICY:
      return [&peers](const geometry_msgs::msg::Point & pt) -> double {
        if (peers.empty()) return 0.0;

        double min_dist_sq = std::numeric_limits<double>::max();
        for (const auto & peer : peers) {
          double dx = pt.x - peer.position.x;
          double dy = pt.y - peer.position.y;
          double dist_sq = dx * dx + dy * dy;
          if (dist_sq < min_dist_sq) {
            min_dist_sq = dist_sq;
          }
        }
        return std::sqrt(min_dist_sq);
      };

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown separation policy");
      return [](const geometry_msgs::msg::Point &) { return 0.0; };
  }
}

PoseWithCost ChooseFrontierGoal::get_best_goal(
    const geometry_msgs::msg::Pose &pose,
    const std::vector<geometry_msgs::msg::Point> &frontier,
    const std::vector<Pose> &peers) {

  // Proximity policy
  std::function<double(const geometry_msgs::msg::Point &)> calculate_proximity;
  calculate_proximity = get_proximity_calculator(pose);

  // Separation policy
  std::function<double(const geometry_msgs::msg::Point &)> calculate_separation;
  calculate_separation = get_separation_calculator(peers);

  // Select best frontier point based on combined cost function
  size_t best_idx = 0;
  double min_cost = std::numeric_limits<double>::max();

  for (size_t i = 0; i < frontier.size(); ++i) {
    double dist_robot = calculate_proximity(frontier[i]);
    double dist_peer = calculate_separation(frontier[i]);

    // Combined cost: minimize distance to the robot while maximizing the
    // distance to the closest teammate
    double cost = (proximity_weight_ * dist_robot) - (separation_weight_ * dist_peer);
    if (cost < min_cost) {
      min_cost = cost;
      best_idx = i;
    }
  }

  // Fill message
  PoseWithCost frontier_goal;
  frontier_goal.pose.position.x = frontier[best_idx].x;
  frontier_goal.pose.position.y = frontier[best_idx].y;

  // Orient toward the goal (vector from robot to frontier)
  double yaw = std::atan2(frontier[best_idx].y - pose.position.y,
                          frontier[best_idx].x - pose.position.x);

  frontier_goal.pose.orientation.z = std::sin(yaw / 2.0);
  frontier_goal.pose.orientation.w = std::cos(yaw / 2.0);
  frontier_goal.cost = min_cost;

  return frontier_goal;
}

} // namespace easynav_multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<easynav_multirobot_exploration::ChooseFrontierGoal>(
      "ChooseFrontierGoal");
}

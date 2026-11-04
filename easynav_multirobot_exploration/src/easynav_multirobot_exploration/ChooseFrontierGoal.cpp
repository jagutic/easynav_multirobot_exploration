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

  cost_markers_ = MarkerArray();

  Marker proximity_marker;
  proximity_marker.header.frame_id = config().blackboard->get<std::string>("map_frame");
  proximity_marker.ns = "proximity_cost";
  proximity_marker.type = Marker::ARROW;
  proximity_marker.action = Marker::ADD;
  proximity_marker.scale.x = 0.06;
  proximity_marker.scale.y = 0.12;
  proximity_marker.scale.z = 0.18;
  proximity_marker.color.r = 0.0F;
  proximity_marker.color.g = 1.0F;
  proximity_marker.color.b = 0.0F;
  proximity_marker.color.a = 1.0F;

  Marker separation_marker = proximity_marker;
  separation_marker.ns = "separation_cost";
  separation_marker.color.r = 1.0F;
  separation_marker.color.g = 0.0F;
  separation_marker.color.b = 0.0F;

  Marker proximity_text = proximity_marker;
  proximity_text.type = Marker::TEXT_VIEW_FACING;
  proximity_text.pose.orientation.w = 1.0;
  proximity_text.scale.x = 0.0;
  proximity_text.scale.y = 0.0;
  proximity_text.scale.z = 0.22;

  Marker separation_text = proximity_text;
  separation_text.ns = "separation_cost";
  separation_text.color = separation_marker.color;

  cost_markers_.markers.push_back(proximity_marker);
  cost_markers_.markers.push_back(proximity_text);
  cost_markers_.markers.push_back(separation_marker);
  cost_markers_.markers.push_back(separation_text);

  cost_markers_pub_ = node_->create_publisher<MarkerArray>(
      "cost_markers", rclcpp::QoS(10).durability_volatile().reliable());

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
  std::vector<Point> frontier_points;
  result = getInput("frontier_points", frontier_points);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No frontier");
    return BT::NodeStatus::FAILURE;
  }
  if (frontier_points.empty()) {
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
    result = getInput("robot_peers_pose", robot_peers);

    if (!result.has_value()) {
      RCLCPP_ERROR(node_->get_logger(), "No peers pose input");
      return BT::NodeStatus::FAILURE;
    }
    if (robot_peers.empty()) {
      RCLCPP_INFO(node_->get_logger(), "No peers for separation");
    }
    break;

  case GOAL_SEPARATION_POLICY:
    result = getInput("robot_peers_goals", robot_peers);

    if (!result.has_value()) {
      RCLCPP_ERROR(node_->get_logger(), "No peers goal input");
      return BT::NodeStatus::FAILURE;
    }
    if (robot_peers.empty()) {
      RCLCPP_INFO(node_->get_logger(), "No peers for separation");
    }
    break;

  default:
    RCLCPP_ERROR(node_->get_logger(),
                 "Wrong separation policy, not choosing frontier goal");
    return BT::NodeStatus::FAILURE;
  }

  // Choose and set the best frontier goal
  auto frontier_goal = get_best_goal(robot_pose, frontier_points, robot_peers);
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

std::function<std::optional<std::pair<double, Pose>>(const geometry_msgs::msg::Point &)> 
ChooseFrontierGoal::get_separation_calculator(const std::vector<Pose> & peers)
{
  switch (separation_policy_) {
    case NO_SEPARATION_POLICY:
      return [](const geometry_msgs::msg::Point &)
          -> std::optional<std::pair<double, Pose>> {
        return std::nullopt;
      };

    case POSITION_SEPARATION_POLICY:
    case GOAL_SEPARATION_POLICY:
        return [&peers](const geometry_msgs::msg::Point & pt)
          -> std::optional<std::pair<double, Pose>> {
        if (peers.empty()) {
          return std::nullopt;
        }

        double min_distance = std::numeric_limits<double>::max();
        Pose closest_peer;
        for (const auto & peer : peers) {
          const double distance = std::hypot(
              pt.x - peer.position.x, pt.y - peer.position.y);
          if (distance < min_distance) {
            min_distance = distance;
            closest_peer = peer;
          }
        }
        return std::pair<double, Pose>{min_distance, closest_peer};
      };

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown separation policy");
      return [](const geometry_msgs::msg::Point &)
          -> std::optional<std::pair<double, Pose>> {
        return std::nullopt;
      };
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
    auto proximity = calculate_proximity(frontier[i]);
    auto separation = calculate_separation(frontier[i]);

    double proximity_cost = proximity_weight_ * proximity;
    double separation_cost = separation ? separation_weight_ * separation->first : 0.0;

    // Minimize distance to the robot
    // Maximize distance to the closest teammate
    double cost = proximity_cost - separation_cost;

    if (cost < min_cost) {
      min_cost = cost;
      best_idx = i;

      // Publish cost markers for visualization
      fill_cost_markers(pose.position, frontier[i], proximity_cost, separation);
      cost_markers_pub_->publish(cost_markers_);
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

void ChooseFrontierGoal::fill_cost_markers(
    const Point &position,
    const Point &goal,
    double proximity_cost,
    const std::optional<std::pair<double, Pose>> &separation) {

  const auto stamp = node_->get_clock()->now();

  // Proximity arrow
  cost_markers_.markers[0].header.stamp = stamp;
  cost_markers_.markers[0].points = {position, goal};

  // Proximity text
  cost_markers_.markers[1].header.stamp = stamp;
  cost_markers_.markers[1].pose.position.x = (position.x + goal.x) / 2.0;
  cost_markers_.markers[1].pose.position.y = (position.y + goal.y) / 2.0;
  cost_markers_.markers[1].pose.position.z = (position.z + goal.z) / 2.0 + 0.25;
  cost_markers_.markers[1].text = std::to_string(proximity_cost);

  if (separation) {
    // Separation arrow
    const auto &peer = separation->second.position;
    const double separation_cost = -(separation_weight_ * separation->first);

    cost_markers_.markers[2].header.stamp = stamp;
    cost_markers_.markers[2].action = Marker::ADD;
    cost_markers_.markers[2].points = {goal, peer};

    // Separation text
    cost_markers_.markers[3].header.stamp = stamp;
    cost_markers_.markers[3].action = Marker::ADD;
    cost_markers_.markers[3].pose.position.x = (goal.x + peer.x) / 2.0;
    cost_markers_.markers[3].pose.position.y = (goal.y + peer.y) / 2.0;
    cost_markers_.markers[3].pose.position.z = (goal.z + peer.z) / 2.0 + 0.25;
    cost_markers_.markers[3].text = std::to_string(separation_cost);

  } else {
    cost_markers_.markers[2].action = Marker::DELETE;
    cost_markers_.markers[3].action = Marker::DELETE;
  }
}

} // namespace easynav_multirobot_exploration

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<easynav_multirobot_exploration::ChooseFrontierGoal>(
      "ChooseFrontierGoal");
}

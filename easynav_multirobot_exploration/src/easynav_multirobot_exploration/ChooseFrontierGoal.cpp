#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"

#include <algorithm>
#include <stdexcept>

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
  node_->declare_parameter<std::string>("planner_plugin", "easynav_costmap_planner/CostmapPlanner");

  node_->get_parameter("proximity_policy", proximity_policy_);
  node_->get_parameter("separation_policy", separation_policy_);
  node_->get_parameter("proximity_weight", proximity_weight_);
  node_->get_parameter("separation_weight", separation_weight_);
  const std::string plugin_name = node_->get_parameter("planner_plugin").as_string();

  if (proximity_weight_ < 0.0 || separation_weight_ < 0.0) {
    RCLCPP_ERROR(node_->get_logger(), "Proximity and separation weights must be non-negative");
    throw std::invalid_argument(
        "Proximity and separation weights must be non-negative");
  }

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

  // Get from pluginlib the planner to compute the real proximity cost
  try {
    planner_ = loader_.createSharedInstance(plugin_name);
    planner_->initialize(planner_node, "proximity_cost_planner", "");

  } catch (const pluginlib::PluginlibException & e) {
    RCLCPP_ERROR(node_->get_logger(), "No se pudo cargar el plugin '%s': %s", plugin_name.c_str(), e.what());
    throw;
  }

  // Markers for cost visualization
  init_cost_markers();
  cost_markers_pub_ = node_->create_publisher<MarkerArray>(
      "cost_markers", rclcpp::QoS(10).durability_volatile().reliable());

  RCLCPP_INFO(node_->get_logger(), "Planner loaded: %s", planner_->getName().c_str());
  RCLCPP_INFO(node_->get_logger(), "Proximity weight: %.2f", proximity_weight_);
  RCLCPP_INFO(node_->get_logger(), "Separation weight: %.2f",
              separation_weight_);
}

void ChooseFrontierGoal::init_cost_markers() {
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

  // Get BB map
  OccupancyGrid frontier_map;
  result = getInput("frontier_map", frontier_map);

  if (!result.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "No frontier map");
    return BT::NodeStatus::FAILURE;
  }
  if (frontier_map.data.empty()) {
    RCLCPP_ERROR(node_->get_logger(), "Frontier map is empty");
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

  // Fill candidates and choose the best frontier goal
  auto candidates = fill_candidates(robot_pose, robot_peers, frontier_points, frontier_map);
  auto frontier_goal = get_best_goal(robot_pose, frontier_map, candidates);
  if (!frontier_goal) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("frontier_goal", *frontier_goal);

  RCLCPP_INFO(node_->get_logger(), "Best frontier candidate, with cost: %.2f",
              frontier_goal->cost);
  return BT::NodeStatus::SUCCESS;
}

std::function<double(const geometry_msgs::msg::Point &)> 
ChooseFrontierGoal::get_proximity_calculator(
  const geometry_msgs::msg::Pose & pose_origin,
  const OccupancyGrid & map)
{
  switch (proximity_policy_) {
    case EUCLIDEAN_PROXIMITY_POLICY:
      return [this, &pose_origin](const geometry_msgs::msg::Point & dest) -> double {
        double euclidean_distance = std::hypot(dest.x - pose_origin.position.x, dest.y - pose_origin.position.y);
        return euclidean_distance * proximity_weight_;
      };

    case REAL_PROXIMITY_POLICY: {
      if (map.data.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Map is empty, cannot compute real proximity");
        return [](const geometry_msgs::msg::Point &) { return -1.0; };
      }

      auto costmap = std::make_shared<easynav::Costmap2D>(map);
      const auto map_frame = map.header.frame_id;

      return [this, &pose_origin, costmap, map_frame](const geometry_msgs::msg::Point & dest) -> double {
        easynav::NavState ns;

        ns.set<easynav::Costmap2D>("map", costmap);

        // Set odom
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = map_frame;
        odom.pose.pose = pose_origin;
        ns.set<nav_msgs::msg::Odometry>("robot_pose", odom);

        // Set goal
        nav_msgs::msg::Goals goals;
        goals.header.frame_id = map_frame;

        geometry_msgs::msg::PoseStamped goal;
        goal.header = goals.header;
        goal.pose = dest;
        goals.goals.push_back(goal);
        ns.set<nav_msgs::msg::Goals>("goals", goals);

        // Get path and compute its length
        planner_->force_update(ns);

        if (!ns.has("path")) {
          RCLCPP_ERROR(node_->get_logger(), "Planner did not produce a path");
          return -1.0;
        }
        const auto & path = ns.get<nav_msgs::msg::Path>("path");
        if (path.poses.empty()) {
          RCLCPP_ERROR(node_->get_logger(), "Planner returned an empty path");
          return -1.0;
        }

        double len = 0.0;
        for (size_t i = 1; i < path.poses.size(); ++i) {
          len += std::hypot(path.poses[i].pose.position.x - path.poses[i-1].pose.position.x,
                            path.poses[i].pose.position.y - path.poses[i-1].pose.position.y);
        }
        return len * proximity_weight_;
      };
    }

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown proximity policy");
      return [](const geometry_msgs::msg::Point &) { return -1.0; };
  }
}

std::function<std::optional<std::pair<double, Pose>>(const geometry_msgs::msg::Point &)> 
ChooseFrontierGoal::get_separation_calculator(
  const std::vector<Pose> & peers_origin)
{
  switch (separation_policy_) {
    case NO_SEPARATION_POLICY:
      return [](const geometry_msgs::msg::Point &)
          -> std::optional<std::pair<double, Pose>> {
        return std::nullopt;
      };

    case POSITION_SEPARATION_POLICY:
    case GOAL_SEPARATION_POLICY:
      return [this, &peers_origin](const geometry_msgs::msg::Point & dest)
          -> std::optional<std::pair<double, Pose>> {
        if (peers_origin.empty()) {
          return std::nullopt;
        }

        double min_distance = std::numeric_limits<double>::max();
        Pose closest_peer;
        for (const auto & peer : peers_origin) {
          const double distance = std::hypot(
              dest.x - peer.position.x, dest.y - peer.position.y);
          if (distance < min_distance) {
            min_distance = distance;
            closest_peer = peer;
          }
        }
        return std::pair<double, Pose>{min_distance * separation_weight_, closest_peer};
      };

    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown separation policy");
      return [](const geometry_msgs::msg::Point &)
          -> std::optional<std::pair<double, Pose>> {
        return std::nullopt;
      };
  }
}

std::vector<ChooseFrontierGoal::Candidate> ChooseFrontierGoal::fill_candidates(
    const Pose &pose,
    const std::vector<Pose> &peers,
    const std::vector<Point> &frontier,
    const OccupancyGrid &map) {

  auto calculate_separation = get_separation_calculator(peers);

  // Fill candidates list
  std::vector<Candidate> candidates;
  candidates.reserve(frontier.size());

  for (size_t i = 0; i < frontier.size(); ++i) {
    auto separation = calculate_separation(frontier[i]);
    double separation_value = separation ? separation->first : 0.0;
    double lower_bound = 0.0;
    
    // Calculate lower bound only when needed
    if (proximity_policy_ == REAL_PROXIMITY_POLICY) {

      // Using euclidean distance as lower bound if calculted for efficiency
      double proximity_euclidean_value = proximity_weight_ * std::hypot(
        frontier[i].x - pose.position.x, frontier[i].y - pose.position.y);

      lower_bound = proximity_euclidean_value - separation_value;
    }

    // Fill candidate with its lower bound, but without proximity yet
    candidates.push_back({frontier[i], lower_bound, separation});
  }

  // Avoid sorting if not neccessary
  if (proximity_policy_ == REAL_PROXIMITY_POLICY) {
    std::stable_sort(candidates.begin(), candidates.end(),
        [](const Candidate &a, const Candidate &b) {
          return a.lower_bound < b.lower_bound;
        });
  }
  return candidates;
}

std::optional<PoseWithCost> ChooseFrontierGoal::get_best_goal(
    const Pose &pose,
    const OccupancyGrid &map,
    std::vector<Candidate> &candidates) {

  auto calculate_proximity = get_proximity_calculator(pose, map);

  // Select best frontier point based on combined cost function
  Point best_point;
  bool found_candidate = false;
  double min_cost = std::numeric_limits<double>::max();

  for (auto &candidate : candidates) {

    // Early exit if the lower bound of the candidate is greater than the minimum cost found
    if (proximity_policy_ == REAL_PROXIMITY_POLICY && candidate.lower_bound >= min_cost) {
      break;
    }

    // Calculate proximity cost for the candidate
    double separation_cost = candidate.separation ? candidate.separation->first : 0.0;
    double proximity_cost = calculate_proximity(candidate.point);
    if (proximity_cost < 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid proximity cost for candidate");
      continue;
    }

    // Minimize distance to the robot
    // Maximize distance to the closest teammate
    double cost = proximity_cost - separation_cost;

    if (cost < min_cost) {
      min_cost = cost;
      best_point = candidate.point;
      found_candidate = true;

      // Publish cost markers for visualization
      fill_cost_markers(
        pose.position, candidate.point, proximity_cost, candidate.separation);
      cost_markers_pub_->publish(cost_markers_);
    }
  }

  if (!found_candidate) {
    RCLCPP_ERROR(node_->get_logger(), "No frontier candidate has a valid proximity cost");
    return std::nullopt;
  }

  // Fill message
  PoseWithCost frontier_goal;
  frontier_goal.pose.position.x = best_point.x;
  frontier_goal.pose.position.y = best_point.y;

  // Orient toward the goal (vector from robot to frontier)
  double yaw = std::atan2(best_point.y - pose.position.y,
                          best_point.x - pose.position.x);

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
    const double separation_cost = - separation->first;

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

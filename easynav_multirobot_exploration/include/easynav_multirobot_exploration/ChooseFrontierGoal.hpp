#ifndef EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP

#include <string>
#include <limits>
#include <cmath>
#include <optional>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "exploration_interfaces/msg/pose_with_cost.hpp"
#include "easynav_costmap_common/Costmap2D.hpp"   // [VERIFICAR] ruta del header
#include "easynav_common/types/NavState.hpp"

#include "behaviortree_cpp/action_node.h"
#include "easynav_core/PlannerMethodBase.hpp"
#include "pluginlib/class_loader.hpp"


namespace easynav_multirobot_exploration
{

#define EUCLIDEAN_PROXIMITY_POLICY 0
#define REAL_PROXIMITY_POLICY 1

#define NO_SEPARATION_POLICY 0
#define POSITION_SEPARATION_POLICY 1
#define GOAL_SEPARATION_POLICY 2

// Type aliases for cleaner ROS 2 message handling
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using nav_msgs::msg::OccupancyGrid;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using exploration_interfaces::msg::PoseWithCost;

/**
 * @class ChooseFrontierGoal
 * @brief A Behavior Tree SyncActionNode that selects the optimal exploration target.
 * * This node evaluates a list of detected frontiers and chooses the best goal
 * (typically the closest one) based on the robot's current pose.
 */
class ChooseFrontierGoal : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for ChooseFrontierGoal.
   * @param name Name of the node as defined in the XML tree.
   * @param conf Configuration containing blackboard and port mapping.
   */
  ChooseFrontierGoal(const std::string & name, const BT::NodeConfig & conf);

  /**
   * @brief The core logic executed when the node is ticked.
   * @return BT::NodeStatus::SUCCESS if a goal is found, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines the input and output ports required for the blackboard.
   * @return BT::PortsList containing robot_pose, frontier_points, and frontier_goal.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<Pose>("robot_pose"),                      // Current robot pose
        BT::InputPort<std::vector<Pose>>("robot_peers_pose"),   // Current poses of peer robots
        BT::InputPort<std::vector<Pose>>("robot_peers_goals"),  // Current goals of peer robots
        BT::InputPort<std::vector<Point>>("frontier_points"),   // Candidate frontier points
        BT::InputPort<OccupancyGrid>("frontier_map"),           // Occupancy grid for frontier evaluation
        BT::OutputPort<PoseWithCost>("frontier_goal")           // Selected frontier goal
      });
  }

private:
  /**
   * @brief Returns a function to calculate proximity based on the selected policy.
   * @param pose The current pose of the robot.
   * @return A function that computes the distance from the robot to a given point.
   */
  std::function<double(const geometry_msgs::msg::Point &)>
    get_proximity_calculator(const geometry_msgs::msg::Pose & pose);

  /**
   * @brief Returns a function to calculate separation based on the selected policy.
   * @param peers A vector of poses representing peer robots.
   * @return A function that computes the distance from a given point to the closest peer robot.
   */
  std::function<std::optional<std::pair<double, Pose>>(const geometry_msgs::msg::Point &)>
    get_separation_calculator(const std::vector<geometry_msgs::msg::Pose> & peers);

  /**
   * @brief Initializes the visualization markers for cost representation.
   * This function sets up the markers used to visualize proximity and separation costs in RViz.
   */
  void init_cost_markers();

  /**
   * @brief Fills the cost markers with the provided cost information.
   * @param pose The current pose of the robot.
   * @param goal The selected frontier goal.
   * @param proximity_cost The computed proximity cost.
   * @param separation The computed separation cost.
   */
  void fill_cost_markers(
    const Point & pose,
    const Point & goal,
    double proximity_cost,
    const std::optional<std::pair<double, Pose>> & separation
  );

  /**
   * @brief Helper function to determine the best frontier point based on a cost function.
   * @param current_pose The current position of the robot.
   * @param peers A vector of poses representing the positions or goals of peer robots, depending on the separation policy.
   * @param frontier A vector of points representing the identified frontiers.
   * @param map The occupancy grid representing the environment.
   * @return The selected Pose to be sent to the navigation stack with its cost.
   */
  PoseWithCost get_best_goal(
    const Pose & pose,
    const std::vector<Pose> & peers = {},
    const std::vector<Point> & frontier,
    const OccupancyGrid & map = {}
  );

  rclcpp::Node::SharedPtr node_;
  MarkerArray cost_markers_;
  rclcpp::Publisher<MarkerArray>::SharedPtr cost_markers_pub_;

  pluginlib::ClassLoader<easynav::PlannerMethodBase> loader_{"easynav_core", "easynav::PlannerMethodBase"};
  std::shared_ptr<easynav::PlannerMethodBase> planner_;
  std::shared_ptr<easynav::Costmap2D> costmap_;

  int proximity_policy_;
  int separation_policy_;
  double proximity_weight_;
  double separation_weight_;
};

} // namespace easynav_multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP

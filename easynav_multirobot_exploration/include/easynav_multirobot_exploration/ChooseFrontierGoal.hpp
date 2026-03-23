#ifndef EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "behaviortree_cpp/action_node.h"


namespace multirobot_exploration
{

// Type aliases for cleaner ROS 2 message handling
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;

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
  ChooseFrontierGoal(const std::string& name, const BT::NodeConfig& conf);

  /**
   * @brief The core logic executed when the node is ticked.
   * @return BT::NodeStatus::SUCCESS if a goal is found, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines the input and output ports required for the blackboard.
   * @return BT::PortsList containing robot_pose, robot_frontier, and frontier_goal.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<Pose>("robot_pose"),                     // Current location of the robot
        BT::InputPort<std::vector<Point>>("robot_frontier"),   // List of candidate frontier points
        BT::OutputPort<Pose>("frontier_goal")                  // The selected navigation target
      });
  }

private:
  /**
   * @brief Helper function to determine the nearest frontier point.
   * @param current_pose The current position of the robot.
   * @param frontier A vector of points representing the identified frontiers.
   * @return The selected Pose to be sent to the navigation stack.
   */
  Pose calc_closest_goal(const Pose& current_pose, const std::vector<Point>& frontier);

  rclcpp::Node::SharedPtr node_; // Pointer to the ROS 2 node for logging and time access
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
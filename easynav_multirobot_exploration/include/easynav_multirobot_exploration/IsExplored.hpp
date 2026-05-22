#ifndef EASYNAV_MULTIROBOT_EXPLORATION__IS_EXPLORED_COMPLETE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__IS_EXPLORED_COMPLETE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace multirobot_exploration
{

#define MAX_EMPTY_FRONTIER_RECEIVED 5

// Type alias for cleaner ROS 2 message handling
using geometry_msgs::msg::Point;

/**
 * @class IsExplored
 * @brief Behavior Tree condition node to evaluate if the exploration mission is finished.
 * * This node checks the current state of the environment (typically evaluating
 * the list of available frontiers). If no valid frontiers remain, it returns SUCCESS,
 * indicating that the area is fully explored. Otherwise, it returns FAILURE.
 */
class IsExplored : public BT::ConditionNode
{
public:
  /**
   * @brief Constructor for the IsExplored condition node.
   * @param name Name of the node as defined in the XML tree.
   * @param conf Configuration containing blackboard and port mapping.
   */
  IsExplored(const std::string & name, const BT::NodeConfig & conf);

  /**
   * @brief Evaluates the exploration completion condition.
   * @return BT::NodeStatus::SUCCESS if exploration is complete, BT::NodeStatus::FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Defines the input and output ports required for the blackboard.
   * @return BT::PortsList containing the frontier list and the occupancy grid to check.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::vector<Point>>("frontier_to_check"), // List of current frontier points
        BT::InputPort<nav_msgs::msg::OccupancyGrid>("map_to_check") // Current map state
      });
  }

private:
  rclcpp::Node::SharedPtr node_; // Pointer to the ROS 2 node for logging purposes
  int empty_frontier_counter;    // Counts how many empty frontiers have arrived
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__IS_EXPLORED_COMPLETE_HPP

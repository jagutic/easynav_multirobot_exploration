#ifndef EASYNAV_MULTIROBOT_EXPLORATION__GO_TO_POSE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__GO_TO_POSE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "easynav_system/GoalManagerClient.hpp"


namespace multirobot_exploration
{

// Type aliases for easier message handling
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

/**
 * @class GoToPose
 * @brief Behavior Tree action node to command the robot to a specific pose.
 * * Inherits from ActionNodeBase to support asynchronous execution (RUNNING state),
 * allowing the robot to move while the tree continues to tick other reactive nodes.
 */
class GoToPose : public BT::ActionNodeBase
{
public:
  /**
   * @brief Constructor for the GoToPose node.
   * @param name Name of the node in the XML description.
   * @param conf Configuration including blackboard and port remapping.
   */
  GoToPose(const std::string& name, const BT::NodeConfig& conf);

  /**
   * @brief Main execution loop for the asynchronous action.
   * @return BT::NodeStatus::SUCCESS if reached, FAILURE if aborted, or RUNNING while moving.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Callback invoked when the node is interrupted or halted by the tree.
   * Used to cancel the active navigation goal in the GoalManager.
   */
  void halt() override;

  /**
   * @brief Defines the required ports for this node.
   * @return BT::PortsList containing the input goal_pose.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<Pose>("goal_pose") // Target pose retrieved from the blackboard
      });
  }

private:
  rclcpp::Node::SharedPtr node_;                    // Pointer to the ROS 2 node for logging and timing
  easynav::GoalManagerClient::SharedPtr nav_client_; // Client to interface with the easynav navigation system
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__GO_TO_POSE_HPP
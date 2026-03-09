#ifndef EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP

#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose.hpp"

// TF
#include <tf2/utils.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace multirobot_exploration
{

/**
 * @class GetPose
 * @brief Behavior Tree action node that retrieves the current pose of the robot.
 * * This node queries the TF2 tree to find the transform between the map frame 
 * and the robot's base frame, returning it as a geometry_msgs::Pose.
 */
class GetPose : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the GetPose BT node.
   * @param name The name of the node in the behavior tree.
   * @param conf The node configuration containing blackboard bindings.
   */
  GetPose(const std::string& name, const BT::NodeConfig& conf);

  /**
   * @brief Main execution function for the BT node.
   * * Looks up the latest TF transform to determine the robot's position 
   * and sets the result on the blackboard.
   * * @return BT::NodeStatus::SUCCESS if the pose was successfully retrieved, 
   * BT::NodeStatus::FAILURE if the TF lookup fails.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Declares the data ports provided by this BT node.
   * @return A list containing the output port for the robot's pose.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::Pose>("pose")
      });
  }

private:
  rclcpp::Node::SharedPtr node_; ///< Shared pointer to the ROS 2 node used for logging.

  std::string tf_prefix_;        ///< Prefix to identify the specific robot's TF frames.
  tf2::BufferCore tf_buffer_;    ///< Core buffer storing the TF tree history.
  tf2_ros::TransformListener tf_listener_; ///< Listener that populates the TF buffer.
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP
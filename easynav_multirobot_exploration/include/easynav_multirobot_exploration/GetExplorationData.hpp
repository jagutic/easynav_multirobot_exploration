#ifndef EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP

#include <string>
#include <vector>
#include <cmath>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose.hpp"

// TF
#include <tf2/utils.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace multirobot_exploration
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;
using visualization_msgs::msg::Marker;


/**
 * @class GetExplorationData
 * @brief Behavior Tree action node that retrieves the current pose of the robot.
 * * This node queries the TF2 tree to find the transform between the map frame
 * and the robot's base frame, returning it as a geometry_msgs::Pose.
 */
class GetExplorationData : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the GetExplorationData BT node.
   * @param name The name of the node in the behavior tree.
   * @param conf The node configuration containing blackboard bindings.
   */
  GetExplorationData(const std::string & name, const BT::NodeConfig & conf);

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
        BT::OutputPort<Pose>("pose"),
        BT::OutputPort<std::vector<Point>>("frontier"),
        BT::OutputPort<nav_msgs::msg::OccupancyGrid>("map")
      });
  }

private:
  /**
   * @brief Obtiene la pose actual del robot (posición y orientación).
   * Generalmente consulta la transformación (TF) entre 'map' y 'base_link'.
   * @return geometry_msgs::msg::Pose con la ubicación actual del robot.
   */
  geometry_msgs::msg::Pose getRobotPose();


  rclcpp::Node::SharedPtr node_;                          ///< Shared pointer to the ROS 2 node used for logging.

  std::mutex frontier_mutex_;                             ///< Mutex to save frontier
  Marker::SharedPtr last_frontier_;                       ///< Last frontier saved from topic
  rclcpp::Subscription<Marker>::SharedPtr frontier_sub_;  ///< Shared pointer to subscriber to frontier topic.

  std::mutex map_mutex_;                                  ///< Mutex to save map
  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;      ///< Last map saved from topic
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;       ///< Shared pointer to subscriber to map topic.

  std::string tf_prefix_;                                 ///< Prefix to identify the specific robot's TF frames.
  tf2::BufferCore tf_buffer_;                             ///< Core buffer storing the TF tree history.
  tf2_ros::TransformListener tf_listener_;                ///< Listener that populates the TF buffer.
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP

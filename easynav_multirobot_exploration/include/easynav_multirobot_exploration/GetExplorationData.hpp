#ifndef EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP

#include <string>
#include <vector>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// TF
#include <tf2/utils.hpp>
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <regex>

namespace multirobot_exploration
{
  
#define GLOBAL_MAP_FRAME "map"

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
        BT::OutputPort<std::vector<Pose>>("peers_pose"),
        BT::OutputPort<std::vector<Point>>("frontier"),
        BT::OutputPort<nav_msgs::msg::OccupancyGrid>("map")
      });
  }

private:
  /**
   * @brief Obtiene la pose desde un frame padre a un frame hijo.
   * @param tf_buffer Buffer de TF2 para realizar las consultas de transformaciones.
   * @param parent_frame Frame de referencia (ej. 'map', 'global_map').
   * @param child_frame Frame destino (ej. 'base_link', otra map de robot).
   * @return geometry_msgs::msg::Pose con la transformación.
   */
  geometry_msgs::msg::Pose getPose(
    const std::string & parent_frame,
    const std::string & child_frame,
    tf2::BufferCore& tf_buffer
  );
  
  /**
   * @brief Obtiene las poses de todos los robots peer desde el GLOBAL_MAP_FRAME.
   * Descubre dinámicamente los frames usando allFramesAsYAML().
   * @return Vector de poses de otros robots.
   */
  std::vector<Pose> getPeersPose();
  
  /**
   * @brief Parsea el YAML de frames para encontrar hijos de un frame padre.
   * @param yaml_str String YAML del árbol de frames.
   * @param parent_frame Frame padre a buscar.
   * @return Vector con los frame_ids que son hijos directos del padre.
   */
  std::vector<std::string> extractChildFrames(const std::string & yaml_str, const std::string & parent_frame);
    
  rclcpp::Node::SharedPtr node_;                          ///< Shared pointer to the ROS 2 node used for logging.
  tf2::BufferCore tf_buffer_;                             ///< Local buffer for TF tree within robot namespace.
  tf2_ros::TransformListener tf_listener_;                ///< Local listener that populates the TF buffer.
  
  rclcpp::Node::SharedPtr global_tf_node_;          ///< Global node (at root namespace) for TF access.
  tf2::BufferCore global_tf_buffer_;                      ///< Global buffer for accessing /tf without namespace.
  std::unique_ptr<tf2_ros::TransformListener> global_tf_listener_;  ///< Global listener with global node.
  
  Marker::SharedPtr last_frontier_;                       ///< Last frontier saved from topic
  rclcpp::Subscription<Marker>::SharedPtr frontier_sub_;  ///< Shared pointer to subscriber to frontier topic.

  nav_msgs::msg::OccupancyGrid::SharedPtr last_map_;      ///< Last map saved from topic
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;       ///< Shared pointer to subscriber to map topic.
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP

#ifndef EASYNAV_MULTIROBOT_EXPLORATION__MUX_MAPS_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__MUX_MAPS_HPP

#include <vector>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "behaviortree_cpp/action_node.h"


namespace multirobot_exploration
{

using nav_msgs::msg::OccupancyGrid;

/**
 * @brief Represents a 2D spatial bounding box.
 * * Used to calculate the global dimensions required to fit multiple 
 * overlapping or disjoint local maps into a single global map.
 */
struct BoundingBox {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
};

/**
 * @class MuxMaps
 * @brief Behavior Tree action node that merges multiple local occupancy grids into a global one.
 * * This node subscribes to multiple map topics (presumably from different robots), 
 * calculates the required global boundaries, and multiplexes them into a single 
 * unified OccupancyGrid that is both published to ROS and set on the BT blackboard.
 */
class MuxMaps : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the MuxMaps BT node.
   * @param name The name of the node in the behavior tree.
   * @param conf The node configuration containing blackboard bindings.
   */
  MuxMaps(const std::string& name, const BT::NodeConfig& conf);

  /**
   * @brief Translates individual robot map coordinates into a unified global frame.
   * @param origin_coord_id The frame identifier to be used as the shared (0,0) origin.
   */
  void translate_robot_coords(std::string origin_coord_id);
  
  /**
   * @brief Main execution function for the BT node.
   * * Triggers the boundary calculation, map multiplexing, and publication of the final map.
   * @return BT::NodeStatus::SUCCESS if the muxing process succeeds, FAILURE otherwise.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Declares the data ports provided by this BT node.
   * @return A list containing the output port for the merged map.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<OccupancyGrid::SharedPtr>("muxed_map")
      });
  }
  
private:
  /**
   * @brief Computes the absolute bounding box encompassing all stored local maps.
   * @return A BoundingBox struct with the extreme min and max spatial coordinates.
   */
  BoundingBox get_global_bounds();

  /**
   * @brief Merges the internal cache of local maps into a single global OccupancyGrid.
   * * This method handles the grid indexing translation and resolves overlapping pixel 
   * values (e.g., preserving known obstacles over unknown space).
   * @param final_map Shared pointer to the pre-allocated map where data will be written.
   */
  void mux(OccupancyGrid::SharedPtr final_map);

  /**
   * @brief Subscription callback to receive and cache incoming local maps.
   * @param map The latest OccupancyGrid message received from a robot.
   */
  void map_callback(const OccupancyGrid::SharedPtr map);

  std::string tf_prefix_;                                         ///< Prefix used to identify specific robot frames.
  std::map<std::string, OccupancyGrid::SharedPtr> maps_;          ///< Cache of the latest local maps, indexed by robot/frame ID.
  std::map<std::string, geometry_msgs::msg::Pose2D> robots_coords_; ///< Local offsets to translate map grids to the global frame.
  
  rclcpp::Node::SharedPtr node_;                                  ///< Shared pointer to the internal ROS 2 node.
  rclcpp::Publisher<OccupancyGrid>::SharedPtr muxed_map_pub_;     ///< Publisher for the final multiplexed global map.
  std::map<std::string, rclcpp::Subscription<OccupancyGrid>::SharedPtr> map_subs_; ///< Map of active ROS 2 subscriptions.

  double PADDING = 0.5;                                           /// Constant value for safety zone around resultant muxed map.
};

} // ns multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__MUX_MAPS_HPP
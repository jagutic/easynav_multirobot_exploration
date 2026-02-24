#ifndef EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP

#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "behaviortree_cpp/action_node.h"

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

// TF
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

/**
 * @brief Helper enumeration to map parameter strings to OpenCV structuring element shapes.
 * * Used to define the spatial behavior of the morphological filters applied to the map.
 */

namespace multirobot_exploration
{

using nav_msgs::msg::OccupancyGrid;

/**
 * @class DetectFrontier
 * @brief Behavior Tree action node that isolates exploration frontiers from an occupancy grid.
 * * Converts the ROS map into OpenCV matrices, applying morphological operations (floodFill, 
 * dilate, morphEx) to cleanly extract the boundaries between reachable free space and unknown zones.
 */
class DetectFrontier : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the DetectFrontier BT node.
   * @param name The name of the node in the behavior tree.
   * @param conf The node configuration containing blackboard bindings and parameters.
   */
  DetectFrontier(const std::string& name, const BT::NodeConfig& conf);

  /**
   * @brief Main execution function for the BT node.
   * * Fetches the map and robot pose, triggers the image-processing frontier detection, 
   * publishes the result for visual debugging, and sets the computed points on the blackboard.
   * @return BT::NodeStatus::SUCCESS if frontiers are detected, FAILURE if inputs are missing or no frontiers exist.
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Declares the data ports provided and required by this BT node.
   * @return A list containing the input map, input pose, and the output vector of frontier points.
   */
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<OccupancyGrid::SharedPtr>("map"),
        BT::InputPort<geometry_msgs::msg::Pose>("robot_pose"),
        BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("frontier")
      });
  }

private:
  /**
   * @brief Converts the raw mathematical frontier coordinates into a ROS 2 Marker for RViz.
   * @param frontier The extracted vector of spatial coordinates outlining the unknown space.
   * @return A visualization_msgs::Marker constructed as a set of points (usually blue).
   */
  visualization_msgs::msg::Marker fill_marker(std::vector<geometry_msgs::msg::Point> frontier);

  /**
   * @brief Core computer vision algorithm translating grid values to binary matrices to extract the boundary.
   * * Applies morphological opening to clear sensor noise, uses floodFill to safely map reachable space 
   * from the robot's footprint, and extracts the dilated intersection with the unknown area.
   * @param map The current OccupancyGrid to be analyzed.
   * @param pose The current robot position used as the seed point for the floodFill algorithm.
   * @return A vector of strictly safe, reachable 2D points sitting directly on the frontier boundary.
   */
  std::vector<geometry_msgs::msg::Point> get_frontier(
  OccupancyGrid::SharedPtr map, geometry_msgs::msg::Pose& pose);

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_; ///< Publisher for RViz visualization.
  rclcpp::Node::SharedPtr node_; ///< Shared pointer to the internal ROS 2 node context.
  std::string tf_prefix_;        ///< Prefix to identify the specific robot's namespace and TF frames.


  // Cross distribution in orden to eliminate noise caused by isolated pixels
  cv::Mat CROSS_KERNEL = (cv::Mat_<char>(3, 3) << 
    -1,  1, -1,
     1, -1,  1,
    -1,  1, -1
  );
  cv::Mat CROSS_MASK = (cv::Mat_<uchar>(3, 3) << 
    0, 1, 0,
    1, 0, 1,
    0, 1, 0
  );
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP
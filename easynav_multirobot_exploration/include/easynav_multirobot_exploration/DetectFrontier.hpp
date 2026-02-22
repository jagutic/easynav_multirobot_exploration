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

enum KernelType {
  RECT,
  CROSS,
  ELLIPSE
};

namespace multirobot_exploration
{

using nav_msgs::msg::OccupancyGrid;

class DetectFrontier : public BT::SyncActionNode
{
public:
  DetectFrontier(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
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
  visualization_msgs::msg::Marker fill_marker(std::vector<geometry_msgs::msg::Point> frontier);
  std::vector<geometry_msgs::msg::Point> get_frontier(
  OccupancyGrid::SharedPtr map, geometry_msgs::msg::Pose& pose);

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_;
  rclcpp::Node::SharedPtr node_;
  std::string tf_prefix_;

  int noise_kernel_size_;
  enum cv::MorphShapes noise_kernel_type_;
  enum cv::MorphShapes dilate_kernel_type_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP
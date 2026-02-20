#ifndef EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "behaviortree_cpp/action_node.h"

// TF
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace multirobot_exploration
{

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::PoseArray;

class DetectFrontier : public BT::SyncActionNode
{
public:
  DetectFrontier(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<OccupancyGrid>("map"),
        BT::OutputPort<PoseArray>("frontier")
      });
  }

private:
  void map_callback(const OccupancyGrid::SharedPtr msg);
  bool get_robot_pose(geometry_msgs::msg::Pose & pose_out);

  rclcpp::Node::SharedPtr node_;
  OccupancyGrid::SharedPtr map_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr map_sub_;

  std::string ns_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__DETECT_FRONTIER_HPP
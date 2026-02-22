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

class GetPose : public BT::SyncActionNode
{
public:
  GetPose(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<geometry_msgs::msg::Pose>("pose")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::string tf_prefix_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__GET_POSE_HPP
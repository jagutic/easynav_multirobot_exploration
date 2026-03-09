#ifndef EASYNAV_MULTIROBOT_EXPLORATION__GO_TO_POSE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__GO_TO_POSE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "easynav_system/GoalManagerClient.hpp"

#define GOAL_RADIUS 0.1

namespace multirobot_exploration
{

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;

class GoToPose : public BT::ActionNodeBase
{
public:
  GoToPose(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  void halt() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<Pose>("goal_pose")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  easynav::GoalManagerClient::SharedPtr nav_client_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__GO_TO_POSE_HPP
#ifndef EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "behaviortree_cpp/action_node.h"

namespace multirobot_exploration
{

using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::Pose;

class ChooseFrontierGoal : public BT::SyncActionNode
{
public:
  ChooseFrontierGoal(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<PoseArray>("frontier"),
        BT::OutputPort<Pose>("frontier_goal")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
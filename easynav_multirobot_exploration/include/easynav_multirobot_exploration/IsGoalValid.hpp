#ifndef EASYNAV_MULTIROBOT_EXPLORATION__IS_GOAL_VALID_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__IS_GOAL_VALID_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace multirobot_exploration
{

using geometry_msgs::msg::Pose;

class IsGoalValid : public BT::ConditionNode
{
public:
  IsGoalValid(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<Pose>("new_goal")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
  Pose last_goal_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__IS_GOAL_VALID_HPP
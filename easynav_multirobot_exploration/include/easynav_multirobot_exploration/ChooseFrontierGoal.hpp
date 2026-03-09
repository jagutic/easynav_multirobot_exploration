#ifndef EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "behaviortree_cpp/action_node.h"


namespace multirobot_exploration
{

using geometry_msgs::msg::Pose;
using visualization_msgs::msg::Marker;
using geometry_msgs::msg::Point;


class ChooseFrontierGoal : public BT::SyncActionNode
{
public:
  ChooseFrontierGoal(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<Pose>("robot_pose"),
        BT::OutputPort<Pose>("frontier_goal")
      });
  }

private:
  Pose calc_closest_goal(const Pose& current_pose);

  std::vector<Point> frontier_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<Marker>::SharedPtr frontier_sub_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__CHOOSE_FRONTIER_GOAL_HPP
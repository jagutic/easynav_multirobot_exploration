#ifndef EASYNAV_MULTIROBOT_EXPLORATION__IS_EXPLORED_COMPLETE_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__IS_EXPLORED_COMPLETE_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "behaviortree_cpp/condition_node.h"

namespace multirobot_exploration
{

using nav_msgs::msg::OccupancyGrid;

class IsExplored : public BT::ConditionNode
{
public:
  IsExplored(const std::string& name, const BT::NodeConfig& conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<OccupancyGrid>("map")
      });
  }

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__IS_EXPLORED_COMPLETE_HPP
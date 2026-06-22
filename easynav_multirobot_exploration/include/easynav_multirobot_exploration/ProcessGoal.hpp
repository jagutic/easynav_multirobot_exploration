#ifndef EASYNAV_MULTIROBOT_EXPLORATION__PROCESS_GOAL_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__PROCESS_GOAL_HPP

#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "exploration_interfaces/msg/pose_with_cost.hpp"
#include "behaviortree_cpp/action_node.h"

namespace easynav_multirobot_exploration
{

using exploration_interfaces::msg::PoseWithCost;
using geometry_msgs::msg::Pose;

class ProcessGoal : public BT::SyncActionNode
{
public:
  ProcessGoal(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<PoseWithCost>("goal_with_cost"),        // Cost of the new goal to evaluate
        BT::OutputPort<Pose>("goal")                          // Output the selected goal
      });
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::optional<double> last_cost_;
  double min_cost_diff_;                                      // Minimum cost difference to change goal
};

} // namespace easynav_multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__PROCESS_GOAL_HPP

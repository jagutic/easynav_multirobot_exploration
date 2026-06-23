#ifndef EASYNAV_MULTIROBOT_EXPLORATION__PROCESS_GOAL_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__PROCESS_GOAL_HPP

#include <string>
#include <optional>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "exploration_interfaces/msg/pose_with_cost.hpp"
#include "behaviortree_cpp/action_node.h"

#define BLACKLIST_TTL 20

namespace easynav_multirobot_exploration
{

using exploration_interfaces::msg::PoseWithCost;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Point;

class ProcessGoal : public BT::SyncActionNode
{
public:
  ProcessGoal(const std::string & name, const BT::NodeConfig & conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<PoseWithCost>("goal_with_cost"),               // Best candidate from ChooseFrontierGoal
        BT::InputPort<std::vector<Point>>("goals_list"),          // Current frontier to check goal validity
        BT::OutputPort<Pose>("goal")                                  // Committed goal for GoToPose
      });
  }

private:
  /**
   * @brief Calculates the minimum improvement threshold based on the current cost.
   * @param cost The current cost of the goal.
   * @return The minimum cost improvement required to switch goals.
   */
  double getImprovedCost(const double cost);

  /**
   * @brief Checks if a given pose is still valid within the current frontier.
   * @param pose The pose to check.
   * @param poses The list of current frontier points.
   * @return True if the pose is still valid, false otherwise.
   */
  bool stillAlive(const Pose & pose, const std::vector<Point> & poses);

  /**
   * @brief Manages the goal candidate, deciding whether to switch or keep the current goal.
   * @param candidate The new goal candidate.
   * @param goals_list The current list of goals.
   * @return The pose of the goal to be committed to.
   */
  Pose manageCandidate(const PoseWithCost & candidate, const std::vector<Point> & goals_list);

  rclcpp::Node::SharedPtr node_;                    // ROS 2 node
  std::optional<PoseWithCost> last_goal_;                       // Last committed goal with cost

  double proximity_threshold_;                      // meters: radius to consider a frontier point is the same
  int repetition_threshold_;                        // Number of consecutive times a goal can be sent before blacklisting
  double improvement_ratio_;                        // switch only if new_cost < current_cost * this value  
};

} // namespace easynav_multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__PROCESS_GOAL_HPP

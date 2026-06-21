#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"
#include "easynav_multirobot_exploration/IsGoalValid.hpp"
#include "easynav_multirobot_exploration/GoToPose.hpp"
#include "easynav_multirobot_exploration/GetExplorationData.hpp"
#include "easynav_multirobot_exploration/IsExplored.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<easynav_multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
  factory.registerNodeType<easynav_multirobot_exploration::IsGoalValid>("IsGoalValid");
  factory.registerNodeType<easynav_multirobot_exploration::GoToPose>("GoToPose");
  factory.registerNodeType<easynav_multirobot_exploration::GetExplorationData>("GetExplorationData");
  factory.registerNodeType<easynav_multirobot_exploration::IsExplored>("IsExplored");

  // Parameters to set in blackboard
  float freq = 1.0f;
  std::string map_frame, robot_frame, odom_frame;
  auto blackboard = BT::Blackboard::create();
  auto node = std::make_shared<rclcpp::Node>("explorer");

  node->declare_parameter("map_frame", "");
  node->declare_parameter("robot_frame", "");
  node->declare_parameter("odom_frame", "");
  node->declare_parameter("freq", freq);

  node->get_parameter("map_frame", map_frame);
  node->get_parameter("robot_frame", robot_frame);
  node->get_parameter("odom_frame", odom_frame);
  node->get_parameter("freq", freq);

  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::string>("map_frame", map_frame);
  blackboard->set<std::string>("robot_frame", robot_frame);
  blackboard->set<std::string>("odom_frame", odom_frame);

  // Create tree using xml file passed as param
  std::string bt_xml_file;
  node->declare_parameter("bt_xml_file", bt_xml_file);
  node->get_parameter("bt_xml_file", bt_xml_file);

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_file, blackboard);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "Error al cargar el XML: %s", e.what());
    return 1;
  }

  rclcpp::Rate rate(freq); // hz
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "\t");
  RCLCPP_INFO(node->get_logger(), "INIT EXPLORER...");

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    // Maxtime is half the period
    executor.spin_all(std::chrono::milliseconds(static_cast<int>(1000 / freq) / 2));
    finish = tree.rootNode()->executeTick() == BT::NodeStatus::SUCCESS;

    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "END EXPLORER...");
  rclcpp::shutdown();
  return 0;
}

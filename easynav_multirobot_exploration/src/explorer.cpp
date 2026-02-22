#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

#include "easynav_multirobot_exploration/MuxMaps.hpp"
#include "easynav_multirobot_exploration/GetPose.hpp"
#include "easynav_multirobot_exploration/DetectFrontier.hpp"
#include "easynav_multirobot_exploration/ChooseFrontierGoal.hpp"
#include "easynav_multirobot_exploration/IsGoalValid.hpp"
#include "easynav_multirobot_exploration/GoToPose.hpp"
#include "easynav_multirobot_exploration/IsExplored.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("explorer");

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<multirobot_exploration::MuxMaps>("MuxMaps");
  factory.registerNodeType<multirobot_exploration::GetPose>("GetPose");
  factory.registerNodeType<multirobot_exploration::DetectFrontier>("DetectFrontier");
  factory.registerNodeType<multirobot_exploration::ChooseFrontierGoal>("ChooseFrontierGoal");
  factory.registerNodeType<multirobot_exploration::IsGoalValid>("IsGoalValid");
  factory.registerNodeType<multirobot_exploration::GoToPose>("GoToPose");
  factory.registerNodeType<multirobot_exploration::IsExplored>("IsExplored");
  
  std::string xml_file;
  node->declare_parameter("bt_xml_file", xml_file);
  node->get_parameter("bt_xml_file", xml_file);

  std::string tf_prefix;
  node->declare_parameter("tf_prefix", tf_prefix);
  node->get_parameter("tf_prefix", tf_prefix);

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", node);
  blackboard->set<std::string>("tf_prefix", tf_prefix);

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(xml_file, blackboard);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Error al cargar el XML: %s", e.what());
    return 1;
  }

  rclcpp::Rate rate(1); // 1 hz
  RCLCPP_INFO(node->get_logger(), "\t");
  RCLCPP_INFO(node->get_logger(), "INIT EXPLORER...");

  bool finish = false;
  while (!finish && rclcpp::ok()) {
    finish = tree.rootNode()->executeTick() != BT::NodeStatus::RUNNING;

    rclcpp::spin_some(node);
    rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "END EXPLORER...");
  rclcpp::shutdown();
  return 0;
}
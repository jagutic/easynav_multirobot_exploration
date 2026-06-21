#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "easynav_multirobot_exploration/utils/MapDataSaver.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<easynav_multirobot_exploration::MapDataSaverNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  rclcpp::Rate rate(1.0); // hz
  while (rclcpp::ok()) {
    executor.spin_some(); 
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
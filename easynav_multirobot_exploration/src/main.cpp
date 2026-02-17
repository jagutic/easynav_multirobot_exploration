#include "easynav_multirobot_exploration/MapsMux.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<easynav::MapsMux>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
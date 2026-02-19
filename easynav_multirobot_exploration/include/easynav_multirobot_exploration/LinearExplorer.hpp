// #ifndef EASYNAV_MULTIROBOT_EXPLORATION__LINEAREXPLORER_HPP_
// #define EASYNAV_MULTIROBOT_EXPLORATION__LINEAREXPLORER_HPP_

// #include <cmath>
// #include <limits>
// #include <memory>
// #include <chrono>
// #include <iostream>

// #include "rclcpp/rclcpp.hpp"
// #include "nav_msgs/msg/occupancy_grid.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
// #include "easynav_system/GoalManagerClient.hpp"

// // TF
// #include "tf2_ros/transform_listener.h"
// #include "tf2_ros/buffer.h"
// #include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


// namespace easynav
// {

// class LinearExplorer : public rclcpp::Node
// {
// public:
//   LinearExplorer();

// private:
//   void control_loop();
//   void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
//   bool get_robot_pose(geometry_msgs::msg::Pose & pose_out);

//   std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

//   rclcpp::TimerBase::SharedPtr timer_;
//   easynav::GoalManagerClient::SharedPtr nav_client_;
//   nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
//   rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

//   geometry_msgs::msg::Point last_goal_pos_;
//   bool first_run_;
//   bool waiting_result_;
// };

// }  // namespace easynav

// #endif  // EASYNAV_MULTIROBOT_EXPLORATION__LINEAREXPLORER_HPP_
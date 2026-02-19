// #include "easynav_multirobot_exploration/LinearExplorer.hpp"
// #include <tf2/utils.h>

// using namespace std::chrono_literals;

// namespace easynav
// {

// LinearExplorer::LinearExplorer()
// : Node("linear_explorer"),
//   first_run_(true),
//   waiting_result_(false)
// {
//   tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
//   tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

//   nav_client_ = std::make_shared<easynav::GoalManagerClient>(shared_from_this());

//   map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
//     "map", 10,
//     std::bind(&LinearExplorer::map_callback, this, std::placeholders::_1));

//   timer_ = this->create_wall_timer(
//     500ms, std::bind(&LinearExplorer::control_loop, this));
// }

// void LinearExplorer::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
// {
//   current_map_ = msg;
// }

// bool LinearExplorer::get_robot_pose(geometry_msgs::msg::Pose & pose_out)
// {
//   try {
//     geometry_msgs::msg::TransformStamped t;
//     t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

//     pose_out.position.x = t.transform.translation.x;
//     pose_out.position.y = t.transform.translation.y;
//     pose_out.position.z = t.transform.translation.z;
//     pose_out.orientation = t.transform.rotation;
//     return true;
//   } catch (const tf2::TransformException & ex) {
//     return false;
//   }
// }

// void LinearExplorer::control_loop()
// {
//   if (!current_map_) {
//     return;
//   }

//   if (waiting_result_) {
//     if (!nav_client_->is_goal_active()) {
//         waiting_result_ = false; 
//     } else {
//         return;
//     }
//   }

//   geometry_msgs::msg::Pose current_pose;
//   if (!get_robot_pose(current_pose)) {
//     return;
//   }

//   double yaw = tf2::getYaw(current_pose.orientation);
//   double linear_dist = 1.0;

//   geometry_msgs::msg::PoseStamped goal_pose;
//   goal_pose.header.frame_id = "map";
//   goal_pose.header.stamp = this->now();

//   goal_pose.pose.position.x = current_pose.position.x + (linear_dist * cos(yaw));
//   goal_pose.pose.position.y = current_pose.position.y + (linear_dist * sin(yaw));
//   goal_pose.pose.orientation = current_pose.orientation;

//   if (nav_client_->send_goal(goal_pose)) {
//       waiting_result_ = true;
//       last_goal_pos_ = goal_pose.pose.position;
//   }
// }

// }
#include "easynav_lazy_localizer/LazyLocalizer.hpp"

namespace easynav
{

using std::placeholders::_1;
using namespace std::chrono_literals;

LazyLocalizer::LazyLocalizer()
{
  // Custom printer registration for NavState to handle Odometry messages.
  // Converts quaternions to RPY for human-readable logging (x, y, yaw).
  NavState::register_printer<nav_msgs::msg::Odometry>(
    [](const nav_msgs::msg::Odometry & odom) {
      std::ostringstream ret;
      double x = odom.pose.pose.position.x;
      double y = odom.pose.pose.position.y;

      tf2::Quaternion q(
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      ret << "Odometry with pose: (x: " << x << ", y: " << y << ", yaw: " << yaw << ")";
      return ret.str();
    });
}

LazyLocalizer::~LazyLocalizer() {}

void
LazyLocalizer::on_initialize()
{
  auto node = get_node();
  // We call it "Lazy" because it doesn't compute localization; it just trusts 
  // and copies the transform already provided by another node (like SLAM).
  RCLCPP_INFO(node->get_logger(), "Initialized Lazy Localizer (trusting existing TFs)");
}

void
LazyLocalizer::update(NavState & nav_state)
{
  nav_msgs::msg::Odometry robot_pose = get_pose();
  if (robot_pose == nav_msgs::msg::Odometry()) {
    return;
  }

  // Set the robot_pose state in the blackboard using the obtained TF.
  // Without publishing it anywhere
  nav_state.set("robot_pose", robot_pose);
}

void
LazyLocalizer::update_rt(NavState & nav_state)
{
  update(nav_state);
}

nav_msgs::msg::Odometry
LazyLocalizer::get_pose()
{
  const auto & tf_info = RTTFBuffer::getInstance()->get_tf_info();

  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    // Attempt to lookup the transform between map and base_footprint using the shared buffer.
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      tf_info.map_frame, tf_info.robot_frame, tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_node()->get_logger(), "LazyLocalizer::update: TF failed: %s", ex.what());
    return nav_msgs::msg::Odometry();
  }

  // Use tf to obtain robot pose
  tf2::Transform tf_robot;
  nav_msgs::msg::Odometry pose;

  tf2::fromMsg(tf_msg.transform, tf_robot);
  pose.pose.pose.position.x = tf_robot.getOrigin().x();
  pose.pose.pose.position.y = tf_robot.getOrigin().y();
  pose.pose.pose.position.z = tf_robot.getOrigin().z();
  pose.pose.pose.orientation = tf2::toMsg(tf_robot.getRotation());
  
  // No velocity estimation in lazy mode.
  pose.twist.twist.linear.x = 0.0;
  pose.twist.twist.linear.y = 0.0;
  pose.twist.twist.linear.z = 0.0;
  pose.twist.twist.angular.x = 0.0;
  pose.twist.twist.angular.y = 0.0;
  pose.twist.twist.angular.z = 0.0;

  // Fill stamps and frames with tf info
  pose.header.stamp = tf_msg.header.stamp;
  pose.header.frame_id = tf_info.map_frame;
  pose.child_frame_id = tf_info.robot_frame;

  return pose;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
// Export as a plugin for the EasyNav Localizer system.
PLUGINLIB_EXPORT_CLASS(easynav::LazyLocalizer, easynav::LocalizerMethodBase)
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

std::expected<void, std::string>
LazyLocalizer::on_initialize()
{
  auto node = get_node();
  // We call it "Lazy" because it doesn't compute localization; it just trusts 
  // and copies the transform already provided by another node (like SLAM).
  RCLCPP_INFO(node->get_logger(), "Initialized Lazy Localizer (trusting existing TFs)");
  return {};
}

void
LazyLocalizer::update(NavState & nav_state)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    // Attempt to lookup the transform between map and base_footprint using the shared buffer.
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      get_tf_prefix() + "map", get_tf_prefix() + "base_footprint", tf2::TimePointZero, tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "LazyLocalizer::update: TF failed: %s", ex.what());
    return;
  }

  // Directly set the robot_pose state in the blackboard using the obtained TF.
  tf2::Transform tf_bft;
  tf2::fromMsg(tf_msg.transform, tf_bft);
  nav_state.set("robot_pose", get_pose_from_tf(tf_bft));
}

void
LazyLocalizer::update_rt(NavState & nav_state)
{
  // Real-time update follows the same logic as standard update,
  // but do not wait with DurationFromSec
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    // Attempt to lookup the transform between map and base_footprint using the shared buffer.
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      get_tf_prefix() + "map", get_tf_prefix() + "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "LazyLocalizer::update: TF failed: %s", ex.what());
    return;
  }

  // Directly set the robot_pose state in the blackboard using the obtained TF.
  tf2::Transform tf_bft;
  tf2::fromMsg(tf_msg.transform, tf_bft);
  nav_state.set("robot_pose", get_pose_from_tf(tf_bft));
}

nav_msgs::msg::Odometry
LazyLocalizer::get_pose_from_tf(tf2::Transform tf)
{
  // Conversion helper: Maps a tf2::Transform into a standard Odometry message.
  // Note: Twist (velocity) data is zeroed as we are only extracting spatial pose.
  nav_msgs::msg::Odometry pose;

  pose.header.stamp = get_node()->now();
  pose.header.frame_id = get_tf_prefix() + "map";
  pose.child_frame_id = get_tf_prefix() + "base_footprint";

  pose.pose.pose.position.x = tf.getOrigin().x();
  pose.pose.pose.position.y = tf.getOrigin().y();
  pose.pose.pose.position.z = tf.getOrigin().z();
  pose.pose.pose.orientation = tf2::toMsg(tf.getRotation());
  
  // No velocity estimation in lazy mode.
  pose.twist.twist.linear.x = 0.0;
  pose.twist.twist.linear.y = 0.0;
  pose.twist.twist.linear.z = 0.0;
  pose.twist.twist.angular.x = 0.0;
  pose.twist.twist.angular.y = 0.0;
  pose.twist.twist.angular.z = 0.0;

  return pose;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
// Export as a plugin for the EasyNav Localizer system.
PLUGINLIB_EXPORT_CLASS(easynav::LazyLocalizer, easynav::LocalizerMethodBase)
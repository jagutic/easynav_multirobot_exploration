// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Implementation of the LazyLocalizer class.


#include "easynav_lazy_localizer/LazyLocalizer.hpp"

namespace easynav
{

using std::placeholders::_1;
using namespace std::chrono_literals;


LazyLocalizer::LazyLocalizer()
{
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

LazyLocalizer::~LazyLocalizer()
{
}

std::expected<void, std::string>
LazyLocalizer::on_initialize()
{
  auto node = get_node();
  RCLCPP_INFO(node->get_logger(), "Initialized Lazy Localizer, (copies already existing tf)");
  return {};
}

void
LazyLocalizer::update(NavState & nav_state)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      get_tf_prefix() + "map", get_tf_prefix() + "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "LazyLocalizer::update: TF failed: %s", ex.what());
    return;
  }

  // Set pose state directly from tf obstained, does no calculations
  tf2::Transform tf_bft;
  tf2::fromMsg(tf_msg.transform, tf_bft);
  nav_state.set("robot_pose", get_pose_from_tf(tf_bft));
}

void
LazyLocalizer::update_rt(NavState & nav_state)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = RTTFBuffer::getInstance()->lookupTransform(
      get_tf_prefix() + "map", get_tf_prefix() + "base_footprint", tf2::TimePointZero, tf2::durationFromSec(0.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(get_node()->get_logger(), "LazyLocalizer::update: TF failed: %s", ex.what());
    return;
  }

  // Set pose state directly from tf obstained, does no calculations
  tf2::Transform tf_bft;
  tf2::fromMsg(tf_msg.transform, tf_bft);
  nav_state.set("robot_pose", get_pose_from_tf(tf_bft));
}

nav_msgs::msg::Odometry
LazyLocalizer::get_pose_from_tf(tf2::Transform tf)
{
  nav_msgs::msg::Odometry pose;

  pose.header.stamp = get_node()->now();
  pose.header.frame_id = get_tf_prefix() + "map";
  pose.child_frame_id = get_tf_prefix() + "base_footprint";

  pose.pose.pose.position.x = tf.getOrigin().x();
  pose.pose.pose.position.y = tf.getOrigin().y();
  pose.pose.pose.position.z = tf.getOrigin().z();
  pose.pose.pose.orientation = tf2::toMsg(tf.getRotation());
  
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
PLUGINLIB_EXPORT_CLASS(easynav::LazyLocalizer, easynav::LocalizerMethodBase)

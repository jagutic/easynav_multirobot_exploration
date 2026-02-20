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
/// \brief Declaration of the LazyLocalizer method.

#ifndef EASYNAV_COSTMAP_LOCALIZER__LAZYLOCALIZER_HPP_
#define EASYNAV_COSTMAP_LOCALIZER__LAZYLOCALIZER_HPP_

#include <expected>
#include <stdexcept>
#include <utility>
#include <fstream>
#include <sstream>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"

#include "nav_msgs/msg/odometry.hpp"

namespace easynav
{

/// \brief A using tf localization method.
class LazyLocalizer : public LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor.
   */
  LazyLocalizer();

  /**
   * @brief Destructor.
   */
  ~LazyLocalizer();

  /**
   * @brief Initializes the localization method.
   *
   * Sets up publishers, subscribers, and prepares the particle filter.
   *
   * @return std::expected<void, std::string> Success or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Update of the localization state.
   *
   * @param nav_state The current navigation state (read/write).
   */
  void update(NavState & nav_state) override;

  /**
   * @brief Update of the localization state in real-time.
   *
   * @param nav_state The current navigation state (read/write).
   */
  void update_rt(NavState & nav_state) override;

  /**
   * @brief Gets the passed pose as an Odometry message.
   *
   * @return A nav_msgs::msg::Odometry message containing the estimated pose.
   */
  nav_msgs::msg::Odometry get_pose_from_tf(tf2::Transform);
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__LAZYLOCALIZER_HPP_

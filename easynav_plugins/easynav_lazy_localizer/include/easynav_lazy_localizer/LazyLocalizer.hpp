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

/**
 * @brief Localization method that pulls the robot pose directly from TF Tree.
 * * This class implements a "lazy" approach: it does not perform any internal
 * probabilistic calculation (like AMCL or EKF). Instead, it acts as a bridge
 * between the TF Tree (provided by SLAM or other sources) and the NavState blackboard.
 */
class LazyLocalizer : public LocalizerMethodBase
{
public:
  /**
   * @brief Constructor. Sets up custom printers for navigation state debugging.
   */
  LazyLocalizer();

  /**
   * @brief Virtual destructor.
   */
  ~LazyLocalizer();

  /**
   * @brief Initializes the localization method.
   * * Verifies TF prefix and prepares the node to listen to existing transforms.
   * * @return std::expected<void, std::string> Success or error message.
   */
  void on_initialize() override;

  /**
   * @brief Standard update loop for the localization state.
   * * Lookups the 'map' to 'base_footprint' transform and updates the NavState pose.
   * * @param nav_state The shared navigation blackboard (read/write).
   */
  void update(NavState & nav_state) override;

  /**
   * @brief Real-time update loop for high-frequency localization updates.
   * * Similar to update(), but intended for high-priority task execution flows.
   * * @param nav_state The shared navigation blackboard (read/write).
   */
  void update_rt(NavState & nav_state) override;

  /**
   * @brief Helper to convert a tf2::Transform into a standard Odometry message.
   * * Fills the position and orientation while zeroing the twist (velocity) components.
   * * @param tf The transform to be converted.
   * @return A nav_msgs::msg::Odometry message containing the robot pose.
   */
  nav_msgs::msg::Odometry get_pose();
};

}  // namespace easynav

#endif  // EASYNAV_COSTMAP_LOCALIZER__LAZYLOCALIZER_HPP_

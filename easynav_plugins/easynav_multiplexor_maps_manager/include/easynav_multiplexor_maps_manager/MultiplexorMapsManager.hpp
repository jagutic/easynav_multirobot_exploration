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
/// \brief Declaration of the MultiplexorMapsManager method.

#ifndef EASYNAV_MULTIPLEXOR_MAPSMANAGER__MULTIPLEXORMAPSMANAGER_HPP_
#define EASYNAV_MULTIPLEXOR_MAPSMANAGER__MULTIPLEXORMAPSMANAGER_HPP_

#include <vector>
#include <expected>

#include <string>
#include <cstring>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <easynav_common/YTSession.hpp>
#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_core/MapsManagerBase.hpp"


namespace easynav
{

using nav_msgs::msg::OccupancyGrid;

/**
 * @brief Represents a 2D spatial bounding box.
 * * Used to calculate the global dimensions required to fit multiple 
 * overlapping or disjoint local maps into a single global map.
 */
struct BoundingBox {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
};


/**
 * @class MultiplexorMapsManager
 * @brief A plugin-based map manager using the COstMap data structure.
 *
 * This manager merges multiple local occupancy grids with grid saved in navstate
 * Subscribes to multiple map topics passed as parameters.
 * Calculates the required global boundaries, and multiplexes them into a single 
 * unified OccupancyGrid that is both published to ROS and set on the nav_state.
 */
class MultiplexorMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  MultiplexorMapsManager();

  /**
   * @brief Destructor.
   */
  ~MultiplexorMapsManager();

  /**
   * @brief Initializes the maps manager.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Updates the internal maps using the current navigation state.
   *
   * Intended to be called periodically. May perform dynamic map updates
   * based on new sensor data or internal state.
   *
   * @param nav_state Current state of the navigation system.
   */
  virtual void update(NavState & nav_state) override;

  /**
   * @brief Translates individual robot map coordinates into a unified global frame.
   * @param origin_coord_id The frame identifier to be used as the shared (0,0) origin.
   */
  void translate_robot_coords(std::string origin_coord_id);
  

private:
  /**
   * @brief Subscription callback to receive and cache incoming local maps.
   * @param map The latest OccupancyGrid message received from a robot.
   */
  void map_callback(const OccupancyGrid::SharedPtr map);

  /**
   * @brief Computes the absolute bounding box encompassing all stored local maps.
   * @return A BoundingBox struct with the extreme min and max spatial coordinates.
   */
  BoundingBox get_global_bounds();

  /**
   * @brief Merges the internal cache of local maps into src, using dst as output global map.
   * * This method handles the grid indexing translation and resolves overlapping pixel 
   * values (e.g., preserving known obstacles over unknown space).
   * @param src fixed map over who all other maps will be muxed
   * @param dst destination for new global map
   */
  void mux(const OccupancyGrid& src, OccupancyGrid& dst);

  std::map<std::string, OccupancyGrid::SharedPtr> maps_;          ///< Cache of the latest local maps, indexed by robot/frame ID.
  std::map<std::string, geometry_msgs::msg::Pose2D> robots_coords_; ///< Local offsets to translate map grids to the global frame.
  
  rclcpp::Publisher<OccupancyGrid>::SharedPtr muxed_map_pub_;     ///< Publisher for the final multiplexed global map.
  std::map<std::string, rclcpp::Subscription<OccupancyGrid>::SharedPtr> map_subs_; ///< Map of active ROS 2 subscriptions.

  double PADDING = 0.5;                                           /// Constant value for safety zone around resultant muxed map.
};

}  // namespace easynav

#endif  // EASYNAV_MULTIPLEXOR_MAPSMANAGER__MULTIPLEXORMAPSMANAGER_HPP_

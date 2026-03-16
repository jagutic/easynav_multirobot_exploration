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
/// \brief Declaration of the FrontierMapsManager method.

#ifndef EASYNAV_MULTIPLEXOR_MAPSMANAGER__FRONTIERMAPSMANAGER_HPP_
#define EASYNAV_MULTIPLEXOR_MAPSMANAGER__FRONTIERMAPSMANAGER_HPP_

#include <expected>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "easynav_costmap_common/costmap_2d.hpp"
#include "easynav_costmap_common/cost_values.hpp"
#include <easynav_common/YTSession.hpp>
#include "easynav_common/RTTFBuffer.hpp"
#include "easynav_core/MapsManagerBase.hpp"


namespace easynav
{

using nav_msgs::msg::OccupancyGrid;


/**
 * @class FrontierMapsManager
 * @brief A plugin-based map manager using the CostMap data structure.
 * Isolates exploration frontiers from an occupancy grid.
 * Converts the ROS map into OpenCV matrices, applying morphological operations (floodFill, 
 * dilate, morphEx) to cleanly extract the boundaries between reachable free space and unknown zones.
 */
class FrontierMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  FrontierMapsManager();

  /**
   * @brief Destructor.
   */
  ~FrontierMapsManager();

  /**
   * @brief Initializes the maps manager.
   *
   * Creates necessary publishers/subscribers and initializes the map instances.
   *
   * @return std::expected<void, std::string> Success or error string.
   */
  virtual void on_initialize() override;

  /**
   * @brief Updates the internal maps using the current navigation state.
   *
   * Intended to be called periodically. May perform dynamic map updates
   * based on new sensor data or internal state.
   *
   * @param nav_state Current state of the navigation system.
   */
  virtual void update(NavState & nav_state) override;

private:
  /**
   * @brief Converts the raw mathematical frontier coordinates into a ROS 2 Marker for RViz.
   * @param frontier The extracted vector of spatial coordinates outlining the unknown space.
   * @return A visualization_msgs::Marker constructed as a set of points (usually blue).
   */
  visualization_msgs::msg::Marker fill_marker(
    const std::vector<geometry_msgs::msg::Point>& frontier);

  /**
   * @brief Core computer vision algorithm translating grid values to binary matrices to extract the boundary.
   * * Applies morphological opening to clear sensor noise, uses floodFill to safely map reachable space 
   * from the robot's footprint, and extracts the dilated intersection with the unknown area.
   * @param map The current OccupancyGrid to be analyzed.
   * @param pose The current robot position used as the seed point for the floodFill algorithm.
   * @return A vector of strictly safe, reachable 2D points sitting directly on the frontier boundary.
   */
  std::vector<geometry_msgs::msg::Point> get_frontier(
    const Costmap2D& map, const nav_msgs::msg::Odometry& pose);

  // Cross distribution in orden to eliminate noise caused by isolated pixels
  cv::Mat CROSS_KERNEL = (cv::Mat_<char>(3, 3) << 
    -1,  1, -1,
     1, -1,  1,
    -1,  1, -1
  );
  cv::Mat CROSS_MASK = (cv::Mat_<uchar>(3, 3) << 
    0, 1, 0,
    1, 0, 1,
    0, 1, 0
  );                                      /// Constant value for safety zone around resultant muxed map.

  float proximity_radius_;
  int obstacle_threshold_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr frontier_pub_; ///< Publisher for RViz visualization.


  /** Internal matrixes to improve efficiency */
  cv::Mat free_space_;
  cv::Mat unknown_space_;
  cv::Mat reachable_mask_;
  cv::Mat reachable_actual_;
  cv::Mat frontiers_;
};

}  // namespace easynav

#endif  // EASYNAV_MULTIPLEXOR_MAPSMANAGER__FRONTIERMAPSMANAGER_HPP_

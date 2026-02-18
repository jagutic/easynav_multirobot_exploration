#ifndef EASYNAV_MAPS_MUX_HPP_
#define EASYNAV_MAPS_MUX_HPP_

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#define MUX_THRESHOLD 50

namespace easynav
{

using nav_msgs::msg::OccupancyGrid;
using geometry_msgs::msg::Pose2D;

struct BoundingBox {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
};

inline int8_t fuse_pixels(int8_t master, int8_t incoming, int8_t threshold);

class MapsMux : public rclcpp::Node
{
public:
  MapsMux();
  OccupancyGrid mux();
  void traslate_robot_coords(std::string origin_coord_id);

private:
  void control_cycle();
  BoundingBox get_global_bounds();
  void map_callback(const OccupancyGrid::SharedPtr map);

  int n_robots_;
  std::string ns_;
  std::map<std::string, Pose2D> robots_coords_;
  std::map<std::string, OccupancyGrid::SharedPtr> maps_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Subs & Pubs
  std::map<std::string, rclcpp::Subscription<OccupancyGrid>::SharedPtr> map_subs_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr muxed_map_pub_;
};

}  // namespace easynav

#endif  // EASYNAV_MAPS_MUX_HPP_
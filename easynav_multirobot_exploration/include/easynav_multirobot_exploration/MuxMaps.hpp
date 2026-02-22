#ifndef EASYNAV_MULTIROBOT_EXPLORATION__MUX_MAPS_HPP
#define EASYNAV_MULTIROBOT_EXPLORATION__MUX_MAPS_HPP

#include <vector>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "behaviortree_cpp/action_node.h"


namespace multirobot_exploration
{

using nav_msgs::msg::OccupancyGrid;

struct BoundingBox {
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
};

class MuxMaps : public BT::SyncActionNode
{
public:
  MuxMaps(const std::string& name, const BT::NodeConfig& conf);
  void translate_robot_coords(std::string origin_coord_id);
  
  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::OutputPort<OccupancyGrid::SharedPtr>("muxed_map")
      });
  }
  
private:
  BoundingBox get_global_bounds();
  void mux(OccupancyGrid::SharedPtr final_map);
  void map_callback(const OccupancyGrid::SharedPtr map);

  std::string tf_prefix_;
  std::map<std::string, OccupancyGrid::SharedPtr> maps_;
  std::map<std::string, geometry_msgs::msg::Pose2D> robots_coords_;
  
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr muxed_map_pub_;
  std::map<std::string, rclcpp::Subscription<OccupancyGrid>::SharedPtr> map_subs_;
};

} // ns multirobot_exploration

#endif // EASYNAV_MULTIROBOT_EXPLORATION__MUX_MAPS_HPP
#ifndef EASYNAV_MULTIROBOT_EXPLORATION__UTILS__MAP_DATA_SAVER_HPP_
#define EASYNAV_MULTIROBOT_EXPLORATION__UTILS__MAP_DATA_SAVER_HPP_

#include <fstream>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace easynav_multirobot_exploration
{

class MapDataSaverNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for the MapDataSaverNode.
   * * Initializes the node, declares parameters, and sets up the map subscription.
   */
  MapDataSaverNode();
  
  /**
   * @brief Destructor for the MapDataSaverNode.
   * * Ensures that the CSV file is properly closed upon node shutdown.
   */
  virtual ~MapDataSaverNode();

private:
  /**
   * @brief Callback function for the map subscription.
   * * Processes incoming OccupancyGrid messages and saves relevant data to a CSV file.
   * @param msg Shared pointer to the received OccupancyGrid message.
   */
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;     // Shared pointer to the map topic subscription.
  std::ofstream csv_file_;                                                    // Output file stream for writing exploration data to CSV.

  const std::string CSV_FILENAME = "exploration_data.csv";
};

}  // namespace easynav_multirobot_exploration

#endif  // EASYNAV_MULTIROBOT_EXPLORATION__UTILS__MAP_DATA_SAVER_HPP_


  
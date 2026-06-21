#include "easynav_multirobot_exploration/utils/MapDataSaver.hpp"

namespace easynav_multirobot_exploration
{

MapDataSaverNode::MapDataSaverNode()
: Node("map_data_saver")
{
  // Get path from parameter, default to home directory
  std::string csv_path;
  this->declare_parameter("csv_path", "~/");
  this->get_parameter("csv_path", csv_path);

  // File with different name for each robot, based on namespace
  std::string ns = this->get_namespace();
  if (ns.size() > 0 && ns[0] == '/') ns = ns.substr(1);
  std::string full_csv_path = (std::filesystem::path(csv_path) / (ns + "_" + CSV_FILENAME)).string();

  csv_file_.open(full_csv_path, std::ios::out);

  // Return to avoid subscribing if the file cannot be opened
  if (!csv_file_.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open CSV: %s", full_csv_path.c_str());
    return;

  } else {
    csv_file_ << "time,cells\n";
    RCLCPP_INFO(this->get_logger(), "CSV file initialized at: %s", full_csv_path.c_str());
  }

  // Subscribe to the map topic
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map_topic", rclcpp::QoS(1).durability_volatile().best_effort(),
    std::bind(&MapDataSaverNode::map_callback, this, std::placeholders::_1));
}

MapDataSaverNode::~MapDataSaverNode()
{
  // Close the CSV file
  if (csv_file_.is_open()) {
    csv_file_.close();
  }
}

void MapDataSaverNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Time
  rclcpp::Time map_time = msg->header.stamp;
  double map_time_sec = map_time.seconds();

  if (map_time_sec == 0.0) {
    return;
  }

  // Cells
  int cells = 0;
  for (const auto & cell : msg->data) {
    if (cell == 0) {
      cells++; // Free cell
    }
  }

  // Write to CSV
  csv_file_ << map_time_sec << "," << cells << "\n";
  csv_file_.flush();
}

}  // namespace easynav_multirobot_exploration


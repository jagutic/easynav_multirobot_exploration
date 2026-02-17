#include "easynav_multirobot_exploration/MapsMux.hpp"

namespace easynav
{

using namespace std::chrono_literals;

MapsMux::MapsMux() : Node("maps_multiplexer")
{
  // Get list of robots existing
  std::vector<std::string> ids;

  this->declare_parameter("robot_ids", ids);
  this->get_parameter("robot_ids", ids);
  RCLCPP_INFO(get_logger(), "%ld robots", ids.size());

  // Get coords for each robot existing
  for (const auto& id : ids) {
    std::string x_key = id + ".x";
    std::string y_key = id + ".y";
    std::string Y_key = id + ".Y";

    this->declare_parameter(x_key, 0.0);
    this->declare_parameter(y_key, 0.0);
    this->declare_parameter(Y_key, 0.0);

    float x, y, Y;
    this->get_parameter(x_key, x);
    this->get_parameter(y_key, y);
    this->get_parameter(Y_key, Y);

    // Add coord to dict
    robots_coords_[id].x = x;
    robots_coords_[id].y = y;
    robots_coords_[id].theta = Y;

    RCLCPP_INFO(get_logger(),
          "ID: %s, X: %.2f, Y: %.2f, theta: %.2f",
          id.c_str(), x, y, Y);

    // Suscribe to every robot map topic
    std::string topic_name = "/" + id + "/map";
    map_subs_[id] = create_subscription<OccupancyGrid>(
        topic_name,
        10,
        std::bind(&MapsMux::map_callback, this, std::placeholders::_1)
    );
  }

  // My robot (ns), acts as origin
  ns_ = std::string(get_namespace());
  ns_ = ns_.substr(1);
  muxed_map_pub_ = create_publisher<OccupancyGrid>("muxed_map", 10);

  RCLCPP_INFO(get_logger(), "Setting origin at robot %s", ns_.c_str());
  traslate_robot_coords(ns_);
}

void
MapsMux::traslate_robot_coords(std::string origin_coord_id)
{
  if (robots_coords_.find(origin_coord_id) == robots_coords_.end()) {
    RCLCPP_ERROR(get_logger(), "%s not in robot list", origin_coord_id.c_str());
    return; 
  }

  Pose2D origin = robots_coords_[origin_coord_id];
  double ox = origin.x;
  double oy = origin.y;
  double oth = origin.theta;

  double c = std::cos(oth);
  double s = std::sin(oth);

  std::map<std::string, Pose2D> coords_transformed;
  for (const auto& [id, p] : robots_coords_) {
      Pose2D p_new;

      // Linear traslation
      double dx = p.x - ox;
      double dy = p.y - oy;

      // Rotation Matrix 2D
      p_new.x = (dx * c) + (dy * s);
      p_new.y = -(dx * s) + (dy * c);

      // New orientation [-PI, PI]
      p_new.theta = p.theta - oth;
      p_new.theta = std::atan2(std::sin(p_new.theta), std::cos(p_new.theta));

      coords_transformed[id] = p_new;
      RCLCPP_INFO(get_logger(),
          "ID: %s, X: %.2f, Y: %.2f, theta: %.2f",
          id.c_str(), p_new.x, p_new.y, p_new.theta);
  }

  // Update
  robots_coords_ = coords_transformed;
}

inline int8_t fuse_pixels(int8_t master, int8_t incoming, int8_t threshold)
{
  // Accept free space when having at most low noise
  if (incoming == 0) {
    return (master < threshold) ? incoming : master;
  }

  // Accept wall always
  if (incoming == 100) return incoming;

  // Accept high noise only if mine is not higher
  if (incoming >= threshold) {
    return (master < incoming) ? incoming : master;
  }

  // Accept low noise only when not knowing anything
  return (master == -1) ? incoming : master;
}

BoundingBox
MapsMux::get_global_bounds()
{
  BoundingBox box;

  for (const auto& [id, map] : maps_) {
    if (robots_coords_.find(id) == robots_coords_.end()) continue;

    Pose2D pose = robots_coords_[id];
    double c = std::cos(pose.theta);
    double s = std::sin(pose.theta);

    double w = map->info.width * map->info.resolution;
    double h = map->info.height * map->info.resolution;
    double ox = map->info.origin.position.x;
    double oy = map->info.origin.position.y;

    double local_corners_x[4] = {ox, ox + w, ox + w, ox};
    double local_corners_y[4] = {oy, oy, oy + h, oy + h};

    for (int i = 0; i < 4; i++) {
      double gx = pose.x + (local_corners_x[i] * c - local_corners_y[i] * s);
      double gy = pose.y + (local_corners_x[i] * s + local_corners_y[i] * c);

      if (gx < box.min_x) box.min_x = gx;
      if (gx > box.max_x) box.max_x = gx;
      if (gy < box.min_y) box.min_y = gy;
      if (gy > box.max_y) box.max_y = gy;
    }
  }

  if (box.min_x > box.max_x) {
      box.min_x = box.min_y = box.max_x = box.max_y = 0.0;
  }

  // Security padding
  double padding = 1.0; 
  box.min_x -= padding;
  box.min_y -= padding;
  box.max_x += padding;
  box.max_y += padding;

  return box;
}

OccupancyGrid MapsMux::mux()
{
  if (maps_.find(ns_) == maps_.end()) {
    RCLCPP_ERROR(get_logger(), "%s map not published yet", ns_.c_str());
    return OccupancyGrid();
  }

  BoundingBox bounds = get_global_bounds();
  OccupancyGrid master_map;
  master_map.header = maps_[ns_]->header;
  
  double res = maps_[ns_]->info.resolution;
  master_map.info.resolution = res;
  master_map.info.origin.position.x = bounds.min_x;
  master_map.info.origin.position.y = bounds.min_y;
  master_map.info.origin.orientation.w = 1.0;

  double width_m = bounds.max_x - bounds.min_x;
  double height_m = bounds.max_y - bounds.min_y;

  master_map.info.width = static_cast<uint32_t>(std::ceil(width_m / res));
  master_map.info.height = static_cast<uint32_t>(std::ceil(height_m / res));

  size_t total_cells = master_map.info.width * master_map.info.height;
  master_map.data.assign(total_cells, -1);

  double m_orig_x = master_map.info.origin.position.x;
  double m_orig_y = master_map.info.origin.position.y;
  int m_w = static_cast<int>(master_map.info.width);
  int m_h = static_cast<int>(master_map.info.height);
  int8_t threshold = MUX_THRESHOLD; 

  for (const auto& [id, map] : maps_) {
    if (robots_coords_.find(id) == robots_coords_.end()) continue;
    if (!map) continue;

    Pose2D pose = robots_coords_[id];
    double c = std::cos(pose.theta);
    double s = std::sin(pose.theta);

    double n_res = map->info.resolution;
    double n_orig_x = map->info.origin.position.x;
    double n_orig_y = map->info.origin.position.y;
    int n_w = static_cast<int>(map->info.width);
    int n_h = static_cast<int>(map->info.height);

    for (int i = 0; i < n_h; ++i) {
      for (int j = 0; j < n_w; ++j) {
        
        int idx = j + (i * n_w);
        int8_t map_value = map->data[idx];

        if (map_value == -1) continue;

        double x_local = n_orig_x + (j * n_res);
        double y_local = n_orig_y + (i * n_res);

        double x_global = pose.x + (x_local * c - y_local * s);
        double y_global = pose.y + (x_local * s + y_local * c);

        int gx = static_cast<int>(std::round((x_global - m_orig_x) / res));
        int gy = static_cast<int>(std::round((y_global - m_orig_y) / res));

        if (gx >= 0 && gx < m_w && gy >= 0 && gy < m_h) {
          int m_idx = gx + (gy * m_w);
          
          int8_t current_val = master_map.data[m_idx];
          int8_t fused = fuse_pixels(current_val, map_value, threshold);
          master_map.data[m_idx] = fused;
        }
      }
    }
  }
  return master_map;
}

void MapsMux::map_callback(const OccupancyGrid::SharedPtr map)
{
  // Get robot id (namespace) form frame id
  std::string frame = map->header.frame_id;
  size_t pos = frame.find('/');
  std::string id = (pos != std::string::npos) ? frame.substr(0, pos) : frame;

  // Save map with its id
  maps_[id] = map;

  OccupancyGrid muxed_map = mux();
  muxed_map_pub_->publish(muxed_map);
  RCLCPP_INFO(get_logger(), "Publishing muxed map");
}

}  // namespace easynav

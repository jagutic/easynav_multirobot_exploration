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

    // RCLCPP_INFO(get_logger(),
    //       "ID: %s, X: %.2f, Y: %.2f, theta: %.2f",
    //       id.c_str(), x, y, Y);

    // Suscribe to every robot map topic
    std::string topic_name = "/" + id + "/map";
    map_subs_[id] = create_subscription<OccupancyGrid>(
        topic_name,
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&MapsMux::map_callback, this, std::placeholders::_1)
    );
  }

  // My robot (ns), acts as origin
  ns_ = std::string(get_namespace());
  ns_ = ns_.substr(1);
  muxed_map_pub_ = create_publisher<OccupancyGrid>(
        "muxed_map", rclcpp::QoS(1).transient_local().reliable());

  RCLCPP_INFO(get_logger(), "Setting origin at robot %s", ns_.c_str());
  translate_robot_coords(ns_);

  timer_ = this->create_wall_timer(
      2s, std::bind(&MapsMux::control_cycle, this));
}

void
MapsMux::control_cycle()
{
  OccupancyGrid muxed_map = mux();
  muxed_map_pub_->publish(muxed_map);

  RCLCPP_INFO(get_logger(),
              "Publishing map with size %dx%d",
              muxed_map.info.height, muxed_map.info.width);
}

void MapsMux::map_callback(const OccupancyGrid::SharedPtr map)
{
  // Get robot id (namespace) form frame id
  std::string frame = map->header.frame_id;
  size_t pos = frame.find('/');
  std::string id = (pos != std::string::npos) ? frame.substr(0, pos) : frame;

  // Save map with its id
  maps_[id] = map;
}

void
MapsMux::translate_robot_coords(std::string origin_coord_id)
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

  OccupancyGrid::SharedPtr local_map = maps_[ns_];
  BoundingBox bounds = get_global_bounds();

  double res = local_map->info.resolution;
  double local_x = local_map->info.origin.position.x;
  double local_y = local_map->info.origin.position.y;
  int w = local_map->info.width;
  int h = local_map->info.height;

  int pad_south = std::max(0, static_cast<int>(std::round((local_y - bounds.min_y) / res)));
  int pad_north = std::max(0, static_cast<int>(std::round((bounds.max_y - (local_y + h * res)) / res)));
  int pad_west  = std::max(0, static_cast<int>(std::round((local_x - bounds.min_x) / res)));
  int pad_east  = std::max(0, static_cast<int>(std::round((bounds.max_x - (local_x + w * res)) / res)));

  // Expand local mat so can mux other maps received
  cv::Mat total_mat;
  cv::Mat local_mat(h, w, CV_8SC1, reinterpret_cast<int8_t*>(local_map->data.data()));

  cv::copyMakeBorder(local_mat, total_mat, 
                     pad_south, pad_north, pad_west, pad_east, 
                     cv::BORDER_CONSTANT, cv::Scalar(-1));
  
  double origin_x = local_x - (pad_west * res);
  double origin_y = local_y - (pad_south * res);

  cv::Mat warped;
  cv::Point2f src[3], dst[3];

  for (const auto& [id, map] : maps_) {
    if (id == ns_) continue;
    if (!map) continue;
    if (robots_coords_.find(id) == robots_coords_.end()) continue;

    Pose2D pose = robots_coords_[id];
    cv::Mat extra_mat(map->info.height, map->info.width, CV_8SC1, reinterpret_cast<int8_t*>(map->data.data()));

    src[0] = {0.f, 0.f};
    src[1] = {static_cast<float>(map->info.width), 0.f};
    src[2] = {0.f, static_cast<float>(map->info.height)};

    float c = std::cos(pose.theta);
    float s = std::sin(pose.theta);
    float ox = map->info.origin.position.x;
    float oy = map->info.origin.position.y;
    float r = map->info.resolution;

    for(int i = 0; i < 3; ++i) {
        float mx = ox + src[i].x * r;
        float my = oy + src[i].y * r;
        float wx = pose.x + (mx * c - my * s);
        float wy = pose.y + (mx * s + my * c);
        dst[i].x = (wx - origin_x) / res;
        dst[i].y = (wy - origin_y) / res;
    }

    cv::Mat M = cv::getAffineTransform(src, dst);
    cv::warpAffine(extra_mat, warped, M, total_mat.size(), cv::INTER_NEAREST, cv::BORDER_CONSTANT, cv::Scalar(-1));
    
    // If there were walls, do not change them
    cv::max(total_mat, warped, total_mat);
  }

  // Transform cvmat back to OccupancyGrid
  OccupancyGrid master_map;

  master_map.header = local_map->header;
  master_map.header.stamp = this->now();
  master_map.info.resolution = res;
  master_map.info.width = total_mat.cols;
  master_map.info.height = total_mat.rows;
  master_map.info.origin.position.x = origin_x;
  master_map.info.origin.position.y = origin_y;
  master_map.info.origin.orientation.w = 1.0;

  size_t size = total_mat.total() * total_mat.elemSize();
  master_map.data.resize(size);
  std::memcpy(master_map.data.data(), total_mat.data, size);

  return master_map;
}

}  // namespace easynav

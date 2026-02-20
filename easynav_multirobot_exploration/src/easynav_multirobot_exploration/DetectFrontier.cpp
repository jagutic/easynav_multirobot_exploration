#include "easynav_multirobot_exploration/DetectFrontier.hpp"

namespace multirobot_exploration
{

DetectFrontier::DetectFrontier(
    const std::string& name,
    const BT::NodeConfig& conf)
  : BT::SyncActionNode(name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_INFO(node_->get_logger(), "** DetectFrontier **");

  map_sub_ = node_->create_subscription<OccupancyGrid>(
    "map", rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&DetectFrontier::map_callback, this, std::placeholders::_1));

  // Get ns without initial '/'
  ns_ = std::string(node_->get_namespace());
  ns_ = ns_.substr(1);
}

BT::NodeStatus
DetectFrontier::tick()
{
  return BT::NodeStatus::SUCCESS;
}

void
DetectFrontier::map_callback(const OccupancyGrid::SharedPtr map)
{
  map_ = map;
}

bool
DetectFrontier::get_robot_pose(geometry_msgs::msg::Pose & pose_out)
{
  geometry_msgs::msg::TransformStamped tf_msg;

  try {
    tf_msg = tf_buffer_.lookupTransform(
      ns_ + "/map", ns_ + "/base_footprint", tf2::TimePointZero);

    pose_out.position.x = tf_msg.transform.translation.x;
    pose_out.position.y = tf_msg.transform.translation.y;
    pose_out.position.z = tf_msg.transform.translation.z;
    pose_out.orientation = tf_msg.transform.rotation;
    return true;

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(node_->get_logger(), "TF failed: %s", ex.what());
    return false;
  }
}


} // namespace multirobot_exploration


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<multirobot_exploration::DetectFrontier>("DetectFrontier");
}
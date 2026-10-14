// FOXGLOVE VIZ - xem foxglove_viz.hpp
#include "foxglove_viz.hpp"

#include <tf2/exceptions.h>

#include <cmath>

FoxgloveViz::FoxgloveViz(rclcpp::Node &node, double lidar_offset_deg, double wheelbase_m,
                         double half_width_m)
    : node_(node) {
  path_pub_ = node.create_publisher<nav_msgs::msg::Path>("/autocar/path", 1);
  path_map_pub_ = node.create_publisher<nav_msgs::msg::Path>("/autocar/path_map", 1);
  odom_pub_ = node.create_publisher<nav_msgs::msg::Odometry>("/autocar/odom", 10);
  car_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/autocar/car", 1);

  // Than xe: hop tu sau truc sau 5 cm toi truoc truc truoc 5 cm, rong 2 x
  // half_width; mui ten do tu truc sau ra truoc 0.4 m = huong xe
  visualization_msgs::msg::Marker body;
  body.header.frame_id = "base_link";
  body.ns = "car";
  body.id = 0;
  body.type = visualization_msgs::msg::Marker::CUBE;
  body.action = visualization_msgs::msg::Marker::ADD;
  body.frame_locked = true;
  body.pose.position.x = 0.5 * wheelbase_m;
  body.pose.position.z = 0.04;
  body.pose.orientation.w = 1.0;
  body.scale.x = wheelbase_m + 0.10;
  body.scale.y = 2.0 * half_width_m;
  body.scale.z = 0.08;
  body.color.r = 0.1f;
  body.color.g = 0.5f;
  body.color.b = 1.0f;
  body.color.a = 0.85f;
  visualization_msgs::msg::Marker arrow = body;
  arrow.id = 1;
  arrow.type = visualization_msgs::msg::Marker::ARROW;
  arrow.pose.position.x = 0.0;
  arrow.pose.position.z = 0.10;
  arrow.scale.x = 0.40;   // dai
  arrow.scale.y = 0.05;   // rong
  arrow.scale.z = 0.05;
  arrow.color.r = 1.0f;
  arrow.color.g = 0.15f;
  arrow.color.b = 0.1f;
  arrow.color.a = 1.0f;
  car_.markers = {body, arrow};
  tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
  static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Khung xe cung REP-103 (0 truoc, 90 trai): laser quay offset quanh Z
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node.now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "laser";
  const double yaw = lidar_offset_deg * M_PI / 180.0;
  t.transform.rotation.z = std::sin(0.5 * yaw);
  t.transform.rotation.w = std::cos(0.5 * yaw);
  static_tf_pub_->sendTransform(t);
}

// PathTracker -> REP-103 (x truoc, y trai, yaw nguoc kim dong ho):
// x = z, y = -x, yaw = -th
void FoxgloveViz::publish_tf(double x, double z, double th) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = node_.now();
  t.header.frame_id = "odom";
  t.child_frame_id = "base_link";
  t.transform.translation.x = z;
  t.transform.translation.y = -x;
  t.transform.rotation.z = std::sin(-0.5 * th);
  t.transform.rotation.w = std::cos(-0.5 * th);
  tf_pub_->sendTransform(t);
  last_tf_ = t;
}

// Them diem p neu cach diem cuoi >= PATH_STEP_M; qua PATH_MAX_PTS thi bo 10 %
// diem cu nhat
void FoxgloveViz::append_pose(nav_msgs::msg::Path &path,
                              const geometry_msgs::msg::PoseStamped &p) {
  const auto &poses = path.poses;
  if (!poses.empty() &&
      std::hypot(p.pose.position.x - poses.back().pose.position.x,
                 p.pose.position.y - poses.back().pose.position.y) < PATH_STEP_M) {
    return;
  }
  path.poses.push_back(p);
  if (path.poses.size() > PATH_MAX_PTS) {
    path.poses.erase(path.poses.begin(), path.poses.begin() + PATH_MAX_PTS / 10);
  }
}

void FoxgloveViz::publish_paths(double v_mps, double yaw_rate) {
  if (last_tf_.header.frame_id.empty()) {
    return;
  }
  nav_msgs::msg::Odometry od;
  od.header = last_tf_.header;
  od.child_frame_id = "base_link";
  od.pose.pose.position.x = last_tf_.transform.translation.x;
  od.pose.pose.position.y = last_tf_.transform.translation.y;
  od.pose.pose.orientation = last_tf_.transform.rotation;
  od.twist.twist.linear.x = v_mps;
  od.twist.twist.angular.z = yaw_rate;
  odom_pub_->publish(od);

  for (auto &m : car_.markers) {
    m.header.stamp = od.header.stamp;
  }
  car_pub_->publish(car_);

  geometry_msgs::msg::PoseStamped ps;
  ps.header = od.header;
  ps.pose = od.pose.pose;
  append_pose(path_, ps);

  // Vi tri xe trong frame map (slam_toolbox). Chua co map (slam:=false, chua
  // du scan) -> bo qua
  try {
    const auto m = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
    geometry_msgs::msg::PoseStamped pm;
    pm.header = m.header;
    pm.pose.position.x = m.transform.translation.x;
    pm.pose.position.y = m.transform.translation.y;
    pm.pose.orientation = m.transform.rotation;
    append_pose(path_map_, pm);
  } catch (const tf2::TransformException &) {
  }

  if (++path_tick_ % 5 == 0) {
    path_.header.stamp = od.header.stamp;
    path_.header.frame_id = "odom";
    path_pub_->publish(path_);
    if (!path_map_.poses.empty()) {
      path_map_.header.stamp = od.header.stamp;
      path_map_.header.frame_id = "map";
      path_map_pub_->publish(path_map_);
    }
  }
}

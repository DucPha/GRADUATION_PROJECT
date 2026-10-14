// FOXGLOVE VIZ - xem foxglove_viz.hpp
#include "foxglove_viz.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>

#include <cmath>
#include <vector>

FoxgloveViz::FoxgloveViz(rclcpp::Node &node, double lidar_offset_deg, double wheelbase_m,
                         double half_width_m)
    : node_(node) {
  pose_pub_ = node.create_publisher<geometry_msgs::msg::PoseStamped>("/autocar/pose", 10);
  path_pub_ = node.create_publisher<nav_msgs::msg::Path>("/autocar/path", 1);
  car_pub_ = node.create_publisher<visualization_msgs::msg::MarkerArray>("/autocar/car", 1);
  static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(node);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // odom -> base_link dong nhat: slam_toolbox tu tim chuyen dong bang so khop
  // scan, map -> odom chinh la vi tri xe. base_link -> laser: khung xe cung
  // REP-103 (0 truoc, 90 trai), laser quay offset quanh Z.
  std::vector<geometry_msgs::msg::TransformStamped> st(2);
  st[0].header.stamp = node.now();
  st[0].header.frame_id = "odom";
  st[0].child_frame_id = "base_link";
  st[0].transform.rotation.w = 1.0;
  st[1].header.stamp = st[0].header.stamp;
  st[1].header.frame_id = "base_link";
  st[1].child_frame_id = "laser";
  const double yaw = lidar_offset_deg * M_PI / 180.0;
  st[1].transform.rotation.z = std::sin(0.5 * yaw);
  st[1].transform.rotation.w = std::cos(0.5 * yaw);
  static_tf_pub_->sendTransform(st);

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
}

void FoxgloveViz::publish() {
  geometry_msgs::msg::TransformStamped m;
  try {
    m = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (const tf2::TransformException &) {
    return;   // slam_toolbox chua chay (slam:=false, khong co LiDAR) / chua co scan
  }

  geometry_msgs::msg::PoseStamped p;
  p.header = m.header;
  p.pose.position.x = m.transform.translation.x;
  p.pose.position.y = m.transform.translation.y;
  p.pose.position.z = m.transform.translation.z;
  p.pose.orientation = m.transform.rotation;
  pose_pub_->publish(p);

  for (auto &mk : car_.markers) {
    mk.header.stamp = m.header.stamp;
  }
  car_pub_->publish(car_);

  // Them diem neu cach diem cuoi >= PATH_STEP_M; qua PATH_MAX_PTS thi bo 10 %
  // diem cu nhat
  const auto &poses = path_.poses;
  if (poses.empty() ||
      std::hypot(p.pose.position.x - poses.back().pose.position.x,
                 p.pose.position.y - poses.back().pose.position.y) >= PATH_STEP_M) {
    path_.poses.push_back(p);
    if (path_.poses.size() > PATH_MAX_PTS) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + PATH_MAX_PTS / 10);
    }
  }
  if (++tick_ % 5 == 0) {
    path_.header.stamp = m.header.stamp;
    path_.header.frame_id = "map";
    path_pub_->publish(path_);
  }
}

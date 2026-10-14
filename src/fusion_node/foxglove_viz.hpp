// ============================================================================
// FOXGLOVE VIZ - vi tri + quy dao THAT cua xe cho Foxglove tren laptop
// ----------------------------------------------------------------------------
// Laptop cung WiFi mo Foxglove -> Open connection -> Foxglove WebSocket ->
// ws://<IP mini PC>:8765 (foxglove_bridge chay trong fusion.launch.py).
// Panel 3D: Display frame = map.
//
// Vi tri xe CHI tu LiDAR (slam_toolbox so khop scan voi phong), KHONG tu lenh
// ga + goc lai: xe dung yen thi dung yen, day xe bang tay thi duong ve theo.
// Goc toa do (0, 0, 0) = cho xe dat luc chay ./run.sh; x = phia truoc xe luc
// do, y = ben trai, z = 0 (LiDAR 2D, san phang).
//
// TF:
//   map -> odom          slam_toolbox (= vi tri xe, vi odom -> base_link co dinh)
//   odom -> base_link    co dinh (dong nhat): khong dung odometry lenh ga
//   base_link -> laser   co dinh, goc lap LiDAR (lidar_mount_offset_deg)
// Topic:
//   /autocar/pose   geometry_msgs/PoseStamped (map) 10 Hz: x, y, z, huong
//   /autocar/path   nav_msgs/Path (map) 2 Hz: duong xe da di
//   /autocar/car    visualization_msgs/MarkerArray: than xe + mui ten huong
//
// Khong dua gi vao dieu khien xe. Goi tu ctrl_group_ cua fusion_node.
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

class FoxgloveViz {
public:
  // lidar_offset_deg: goc khung xe = goc scan + offset (lidar_node.hpp);
  // wheelbase_m / half_width_m: kich thuoc ve than xe
  FoxgloveViz(rclcpp::Node &node, double lidar_offset_deg, double wheelbase_m,
              double half_width_m);

  // Goi 10 Hz: /autocar/pose, /autocar/car moi lan, /autocar/path moi 5 lan
  void publish();

private:
  rclcpp::Node &node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr car_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  visualization_msgs::msg::MarkerArray car_;
  nav_msgs::msg::Path path_;
  long tick_ = 0;

  // 1 diem moi 3 cm, toi da 5000 diem (bo diem cu) cho nhe WiFi
  static constexpr double PATH_STEP_M = 0.03;
  static constexpr size_t PATH_MAX_PTS = 5000;
};

// ============================================================================
// FOXGLOVE VIZ - quy dao xe + TF cho Foxglove tren laptop (CHI DE XEM)
// ----------------------------------------------------------------------------
// Laptop cung WiFi mo Foxglove -> Open connection -> Foxglove WebSocket ->
// ws://<IP mini PC>:8765 (foxglove_bridge chay trong fusion.launch.py).
// Panel 3D: Display frame = map (co slam_toolbox) hoac odom.
//
// Topic / TF:
//   TF odom -> base_link (truc sau)  odometry PathTracker (lenh ga + goc lai)
//   TF base_link -> laser (tinh)     goc lap LiDAR (lidar_mount_offset_deg)
//   /autocar/car       visualization_msgs/MarkerArray: than xe + mui ten huong
//                      gan vao base_link (xe dung yen thi base_link trung
//                      goc toa do, chi co truc TF thi khong nhin thay xe)
//   /autocar/odom      nav_msgs/Odometry 10 Hz
//   /autocar/path      nav_msgs/Path, frame odom: TROI dan (12/10: ~30 m
//                      sau vai vong, heading tu goc lai khong du chinh xac)
//   /autocar/path_map  nav_msgs/Path, frame map: vi tri xe theo slam_toolbox
//                      (LiDAR sua troi), chi co khi slam:=true
//
// Khong dua gi vao dieu khien xe. Moi ham goi tu CUNG 1 callback group
// (ctrl_group_ cua fusion_node) nen khong can khoa.
// ============================================================================
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>

class FoxgloveViz {
public:
  // lidar_offset_deg: goc khung xe = goc scan + offset (lidar_node.hpp);
  // wheelbase_m / half_width_m: kich thuoc ve than xe
  FoxgloveViz(rclcpp::Node &node, double lidar_offset_deg, double wheelbase_m,
              double half_width_m);

  // Vi tri truc sau theo PathTracker (X phai, Z truoc, th > 0 = quay phai)
  // -> TF odom -> base_link. Goi ~50 Hz (slam_toolbox can TF luc moi scan).
  void publish_tf(double x, double z, double th);

  // /autocar/odom moi lan goi; /autocar/path, /autocar/path_map moi 5 lan.
  // Goi 10 Hz. v_mps / yaw_rate (rad/s, > 0 = quay trai) chi de ve Plot.
  void publish_paths(double v_mps, double yaw_rate);

private:
  static void append_pose(nav_msgs::msg::Path &path,
                          const geometry_msgs::msg::PoseStamped &p);

  rclcpp::Node &node_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_map_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr car_pub_;
  visualization_msgs::msg::MarkerArray car_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  geometry_msgs::msg::TransformStamped last_tf_; // odom -> base_link moi nhat
  nav_msgs::msg::Path path_;
  nav_msgs::msg::Path path_map_;
  long path_tick_ = 0;

  // 1 diem moi 5 cm, toi da 3000 diem (bo diem cu) cho nhe WiFi
  static constexpr double PATH_STEP_M = 0.05;
  static constexpr size_t PATH_MAX_PTS = 3000;
};

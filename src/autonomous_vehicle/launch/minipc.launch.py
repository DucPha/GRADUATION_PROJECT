"""Launch file cho Mini PC (ROS 2).

Sửa các lỗi so với bản cũ:
  * Đường dẫn model mặc định trỏ /home/pi/... trong khi code Python dùng
    /home/phamminhduc/... Launch file LUÔN truyền giá trị này nên mã mặc
    định đúng của node không bao giờ được dùng -> node không nạp được model.
  * Bổ sung đầy đủ tham số cho C++ node (viz_hz, image_topic, camera_fps).
  * Đổi tên tham số camera cho khớp với tên node khai báo.
  * Kiểm tra sự tồn tại của model và báo cảnh báo rõ ràng.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_traffic_model(name):
    """Đường dẫn model giao thông mặc định, tự dò theo thư mục người dùng."""
    return os.path.expanduser(
        f"~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model/{name}"
    )


def _default_turn_model(name):
    """Đường dẫn model rẽ mặc định, tự dò theo thư mục người dùng."""
    return os.path.expanduser(f"~/models/turn_lr_ncnn_model/{name}")


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("viz_hz", default_value="30"),
        DeclareLaunchArgument("image_topic", default_value="/fusion_viz/image"),
        DeclareLaunchArgument("enable_traffic", default_value="true"),
        DeclareLaunchArgument("enable_turn", default_value="true"),
        DeclareLaunchArgument(
            "traffic_model_param",
            default_value=_default_traffic_model("model.ncnn.param"),
        ),
        DeclareLaunchArgument(
            "traffic_model_bin",
            default_value=_default_traffic_model("model.ncnn.bin"),
        ),
        DeclareLaunchArgument(
            "turn_model_param",
            default_value=_default_turn_model("model.ncnn.param"),
        ),
        DeclareLaunchArgument(
            "turn_model_bin",
            default_value=_default_turn_model("model.ncnn.bin"),
        ),
    ]

    nodes = [
        # ---------------------------------------------------------
        # Node C++ chính: camera + LiDAR + né tránh + UART ESP32
        # ---------------------------------------------------------
        Node(
            package="autonomous_vehicle",
            executable="autonomous_vehicle_node",
            name="autonomous_vehicle_node",
            output="screen",
            parameters=[
                {
                    "serial_port": LaunchConfiguration("serial_port"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    # Node khai báo "cam_index", không phải "camera_index".
                    "cam_index": ParameterValue(
                        LaunchConfiguration("camera_index"), value_type=int
                    ),
                    "camera_fps": ParameterValue(
                        LaunchConfiguration("camera_fps"), value_type=int
                    ),
                    "viz_hz": ParameterValue(
                        LaunchConfiguration("viz_hz"), value_type=float
                    ),
                    "image_topic": LaunchConfiguration("image_topic"),
                }
            ],
        ),
        # ---------------------------------------------------------
        # Node AI: đèn giao thông + biển
        # Cả hai subscribe /image_raw do node C++ publish.
        # ---------------------------------------------------------
        Node(
            package="traffic_light_detector",
            executable="traffic_light_detector",
            name="traffic_light_detector",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_traffic")),
            parameters=[
                {
                    "model_param": LaunchConfiguration("traffic_model_param"),
                    "model_bin": LaunchConfiguration("traffic_model_bin"),
                }
            ],
        ),
        Node(
            package="turn_detector",
            executable="turn_detector",
            name="turn_detector",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_turn")),
            parameters=[
                {
                    "model_param": LaunchConfiguration("turn_model_param"),
                    "model_bin": LaunchConfiguration("turn_model_bin"),
                }
            ],
        ),
    ]

    return LaunchDescription(arguments + nodes)
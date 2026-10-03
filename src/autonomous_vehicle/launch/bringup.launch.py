"""System bringup launch cho Autonomous Vehicle (nguồn tham số duy nhất).

Chạy toàn bộ hệ thống:
- C++ fusion_viz_node : camera + LiDAR + obstacle avoidance + UART ESP32
- traffic_light_detector : nhận dạng đèn giao thông / biển (NCNN)
- turn_detector         : nhận dạng mũi tên rẽ (NCNN)
- rviz2 (tuỳ chọn)
"""

import os
import glob

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _auto_detect_camera():
    """Tự động quét và tìm cổng /dev/video* khả dụng trên máy để truyền cho C++"""
    video_devs = glob.glob('/dev/video*')
    indexes = []
    for dev in video_devs:
        try:
            indexes.append(int(dev.replace('/dev/video', '')))
        except ValueError:
            continue
    
    if indexes:
        # Trả về cổng có index nhỏ nhất (ví dụ có video1 và video2 -> chọn 1)
        return str(min(indexes))
    return "0" # Fallback nếu không tìm thấy


def _default_traffic_model(name):
    env_dir = os.environ.get("TRAFFIC_MODEL_DIR")
    if env_dir:
        return os.path.join(os.path.expandvars(os.path.expanduser(env_dir)), name)
    
    try:
        pkg_share = get_package_share_directory("traffic_light_detector")
        model_path = os.path.join(pkg_share, "models", name)
        if os.path.exists(model_path):
            return model_path
    except Exception:
        pass
    
    return os.path.expanduser(
        f"~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model/{name}"
    )


def _default_turn_model(name):
    env_dir = os.environ.get("TURN_MODEL_DIR")
    if env_dir:
        return os.path.join(os.path.expandvars(os.path.expanduser(env_dir)), name)
    
    try:
        pkg_share = get_package_share_directory("turn_detector")
        model_path = os.path.join(pkg_share, "models", name)
        if os.path.exists(model_path):
            return model_path
    except Exception:
        pass
    
    return os.path.expanduser(f"~/models/turn_lr_ncnn_model/{name}")


def generate_launch_description():
    args = [
        DeclareLaunchArgument("namespace", default_value="", description="Namespace cho toàn bộ node"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Dùng đồng hồ mô phỏng"),
        DeclareLaunchArgument("log_level", default_value="info", description="Mức log của các node"),

        # Hardware
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="230400"),
        
        # GỌI HÀM TỰ DÒ CAMERA Ở ĐÂY THAY VÌ TRUYỀN -1
        DeclareLaunchArgument(
            "camera_index", default_value=_auto_detect_camera(),
            description="Chỉ số V4L2 của camera đã được file launch tự động quét và nhận diện"
        ),
        
        DeclareLaunchArgument("camera_fps", default_value="30"),
        DeclareLaunchArgument("camera_auto_exposure", default_value="true"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("lidar_mount_offset_deg", default_value="-90.0"),

        # Visualization
        DeclareLaunchArgument("viz_hz", default_value="10.0"),
        DeclareLaunchArgument("image_topic", default_value="/fusion_viz/image"),
        DeclareLaunchArgument("raw_image_topic", default_value="/image_raw"),
        DeclareLaunchArgument("raw_image_hz", default_value="5.0"),
        DeclareLaunchArgument("enable_rviz", default_value="false"),

        # Topic AI
        DeclareLaunchArgument("traffic_light_decision_topic", default_value="/traffic_light/decision"),
        DeclareLaunchArgument("turn_detector_decision_topic", default_value="/turn_detector/decision"),
        DeclareLaunchArgument("traffic_debug_image_topic", default_value="/traffic_light/image_debug"),
        DeclareLaunchArgument("turn_debug_image_topic", default_value="/turn_detector/image_debug"),

        # Lách tay theo biển rẽ
        DeclareLaunchArgument("turn_blend_px", default_value="30"),
        DeclareLaunchArgument("turn_speed_x10", default_value="40"),
        DeclareLaunchArgument("speed_normal_x10", default_value="0"),

        # Watchdog
        DeclareLaunchArgument("camera_stale_timeout_s", default_value="1.0"),
        DeclareLaunchArgument("lidar_stale_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("ai_timeout_s", default_value="2.0"),

        # AI detectors
        DeclareLaunchArgument("enable_traffic", default_value="true"),
        DeclareLaunchArgument("enable_turn", default_value="true"),
        DeclareLaunchArgument("traffic_model_param", default_value=_default_traffic_model("model.ncnn.param")),
        DeclareLaunchArgument("traffic_model_bin", default_value=_default_traffic_model("model.ncnn.bin")),
        DeclareLaunchArgument("turn_model_param", default_value=_default_turn_model("model.ncnn.param")),
        DeclareLaunchArgument("turn_model_bin", default_value=_default_turn_model("model.ncnn.bin")),
        DeclareLaunchArgument("traffic_infer_hz", default_value="30.0"),
        DeclareLaunchArgument("turn_infer_hz", default_value="30.0"),
    ]

    cpp_node = Node(
        package="autonomous_vehicle",
        executable="autonomous_vehicle_node",
        name="autonomous_vehicle_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "cam_index": ParameterValue(LaunchConfiguration("camera_index"), value_type=int),
            "camera_fps": ParameterValue(LaunchConfiguration("camera_fps"), value_type=int),
            "camera_auto_exposure": ParameterValue(LaunchConfiguration("camera_auto_exposure"), value_type=bool),
            "viz_hz": ParameterValue(LaunchConfiguration("viz_hz"), value_type=float),
            "image_topic": LaunchConfiguration("image_topic"),
            "raw_image_topic": LaunchConfiguration("raw_image_topic"),
            "raw_image_hz": ParameterValue(LaunchConfiguration("raw_image_hz"), value_type=float),
            "lidar_mount_offset_deg": ParameterValue(LaunchConfiguration("lidar_mount_offset_deg"), value_type=float),
            "camera_stale_timeout_s": ParameterValue(LaunchConfiguration("camera_stale_timeout_s"), value_type=float),
            "lidar_stale_timeout_s": ParameterValue(LaunchConfiguration("lidar_stale_timeout_s"), value_type=float),
            "ai_timeout_s": ParameterValue(LaunchConfiguration("ai_timeout_s"), value_type=float),
            "traffic_light_topic": LaunchConfiguration("traffic_light_decision_topic"),
            "turn_detector_topic": LaunchConfiguration("turn_detector_decision_topic"),
            "turn_blend_px": ParameterValue(LaunchConfiguration("turn_blend_px"), value_type=int),
            "turn_speed_x10": ParameterValue(LaunchConfiguration("turn_speed_x10"), value_type=int),
            "speed_normal_x10": ParameterValue(LaunchConfiguration("speed_normal_x10"), value_type=int),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=2.0,
    )

    traffic_node = Node(
        package="traffic_light_detector",
        executable="traffic_light_detector",
        name="traffic_light_detector",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_traffic")),
        parameters=[{
            "model_param": LaunchConfiguration("traffic_model_param"),
            "model_bin": LaunchConfiguration("traffic_model_bin"),
            "infer_hz": ParameterValue(LaunchConfiguration("traffic_infer_hz"), value_type=float),
            "image_topic": LaunchConfiguration("raw_image_topic"),
            "decision_topic": LaunchConfiguration("traffic_light_decision_topic"),
            "debug_image_topic": LaunchConfiguration("traffic_debug_image_topic"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=3.0,
    )

    turn_node = Node(
        package="turn_detector",
        executable="turn_detector",
        name="turn_detector",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_turn")),
        parameters=[{
            "model_param": LaunchConfiguration("turn_model_param"),
            "model_bin": LaunchConfiguration("turn_model_bin"),
            "infer_hz": ParameterValue(LaunchConfiguration("turn_infer_hz"), value_type=float),
            "image_topic": LaunchConfiguration("raw_image_topic"),
            "decision_topic": LaunchConfiguration("turn_detector_decision_topic"),
            "debug_image_topic": LaunchConfiguration("turn_debug_image_topic"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=3.0,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", os.path.join(get_package_share_directory("autonomous_vehicle"), "rviz", "autonomous.rviz")],
    )

    nodes = [
        cpp_node,
        TimerAction(period=2.0, actions=[
            LogInfo(msg="Starting AI detection nodes..."),
            traffic_node,
            turn_node,
        ]),
        TimerAction(period=4.0, actions=[
            LogInfo(msg="Starting RViz2..."),
            rviz_node,
        ]),
    ]

    return LaunchDescription(args + nodes)
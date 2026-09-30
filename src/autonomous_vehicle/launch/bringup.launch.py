"""System bringup launch for Autonomous Vehicle.

Launches all nodes with proper parameters and dependencies:
- C++ fusion_viz_node (camera + LiDAR + obstacle avoidance + UART)
- traffic_light_detector (NCNN traffic light/sign detection)
- turn_detector (NCNN turn direction detection)
- Optional: rviz2 for visualization
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def _default_traffic_model(name):
    return os.path.expanduser(
        f"~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model/{name}"
    )


def _default_turn_model(name):
    return os.path.expanduser(f"~/models/turn_lr_ncnn_model/{name}")


def generate_launch_description():
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    args = [
        # System
        DeclareLaunchArgument("namespace", default_value="", description="Namespace for all nodes"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="Use simulation time"),
        DeclareLaunchArgument("log_level", default_value="info", description="Log level"),

        # Hardware
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0", description="ESP32 serial port"),
        DeclareLaunchArgument("camera_index", default_value="0", description="V4L2 camera index"),
        DeclareLaunchArgument("camera_fps", default_value="120", description="Camera FPS"),
        DeclareLaunchArgument("scan_topic", default_value="/scan", description="LiDAR scan topic"),

        # Visualization
        DeclareLaunchArgument("viz_hz", default_value="120.0", description="Visualization rate (Hz)"),
        DeclareLaunchArgument("image_topic", default_value="/fusion_viz/image", description="Output image topic"),
        DeclareLaunchArgument("enable_rviz", default_value="false", description="Launch RViz2"),

        # AI Models
        DeclareLaunchArgument("enable_traffic", default_value="true", description="Enable traffic light detector"),
        DeclareLaunchArgument("enable_turn", default_value="true", description="Enable turn detector"),
        DeclareLaunchArgument("traffic_model_param", default_value=_default_traffic_model("model.ncnn.param")),
        DeclareLaunchArgument("traffic_model_bin", default_value=_default_traffic_model("model.ncnn.bin")),
        DeclareLaunchArgument("turn_model_param", default_value=_default_turn_model("model.ncnn.param")),
        DeclareLaunchArgument("turn_model_bin", default_value=_default_turn_model("model.ncnn.bin")),

        # AI Parameters
        DeclareLaunchArgument("traffic_infer_hz", default_value="30.0", description="Traffic detector inference rate"),
        DeclareLaunchArgument("turn_infer_hz", default_value="30.0", description="Turn detector inference rate"),
        DeclareLaunchArgument("ai_timeout_s", default_value="2.0", description="AI decision timeout (seconds)"),

        # Serial
        DeclareLaunchArgument("baudrate", default_value="230400", description="UART baudrate"),
    ]

    # =========================================================================
    # Nodes
    # =========================================================================

    # C++ Main Node: Camera + LiDAR + Obstacle Avoidance + UART
    cpp_node = Node(
        package="autonomous_vehicle",
        executable="autonomous_vehicle_node",
        name="autonomous_vehicle_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "serial_port": LaunchConfiguration("serial_port"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "cam_index": LaunchConfiguration("camera_index"),
            "camera_fps": LaunchConfiguration("camera_fps"),
            "viz_hz": LaunchConfiguration("viz_hz"),
            "image_topic": LaunchConfiguration("image_topic"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=2.0,
    )

    # Traffic Light Detector (Python NCNN)
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
            "infer_hz": LaunchConfiguration("traffic_infer_hz"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=3.0,
    )

    # Turn Detector (Python NCNN)
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
            "infer_hz": LaunchConfiguration("turn_infer_hz"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=3.0,
    )

    # RViz2 (optional)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=[
            "-d", os.path.join(
                os.getenv("HOME", "/home/phamminhduc"),
                "Documents/GRADUATION_PROJECT/src/autonomous_vehicle/rviz/autonomous.rviz"
            )
        ],
    )

    # =========================================================================
    # Startup sequence with delays to ensure proper initialization order
    # =========================================================================
    nodes = [
        # Start C++ node first (it publishes /image_raw for AI nodes)
        cpp_node,

        # Start AI detectors after C++ node is up (2s delay)
        TimerAction(period=2.0, actions=[
            LogInfo(msg="Starting AI detection nodes..."),
            traffic_node,
            turn_node,
        ]),

        # Start RViz last
        TimerAction(period=4.0, actions=[
            LogInfo(msg="Starting RViz2..."),
            rviz_node,
        ]),
    ]

    return LaunchDescription(args + nodes)
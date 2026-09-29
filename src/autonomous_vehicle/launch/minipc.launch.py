from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument("enable_traffic", default_value="true"),
        DeclareLaunchArgument("enable_turn", default_value="true"),
        DeclareLaunchArgument(
            "traffic_model_param",
            default_value=(
                "/home/pi/yolo_ws/runs/detect/traffic_all_red_turn_e10/"
                "weights/best_ncnn_model/model.ncnn.param"
            ),
        ),
        DeclareLaunchArgument(
            "traffic_model_bin",
            default_value=(
                "/home/pi/yolo_ws/runs/detect/traffic_all_red_turn_e10/"
                "weights/best_ncnn_model/model.ncnn.bin"
            ),
        ),
        DeclareLaunchArgument(
            "turn_model_param",
            default_value="/home/pi/models/turn_lr_ncnn_model/model.ncnn.param",
        ),
        DeclareLaunchArgument(
            "turn_model_bin",
            default_value="/home/pi/models/turn_lr_ncnn_model/model.ncnn.bin",
        ),
    ]

    nodes = [
        Node(
            package="autonomous_vehicle",
            executable="autonomous_vehicle_node",
            name="autonomous_vehicle_node",
            output="screen",
            parameters=[
                {
                    "serial_port": LaunchConfiguration("serial_port"),
                    "scan_topic": LaunchConfiguration("scan_topic"),
                    "cam_index": ParameterValue(
                        LaunchConfiguration("camera_index"), value_type=int
                    ),
                }
            ],
        ),
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
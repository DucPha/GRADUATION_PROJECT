from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("demo", default_value="false"),
        DeclareLaunchArgument("domain_id", default_value="0"),
    ]

    node = Node(
        package="autonomous_vehicle_gui",
        executable="autonomous_vehicle_gui_node",
        name="autonomous_vehicle_gui_node",
        output="screen",
        parameters=[
            {
                "demo": LaunchConfiguration("demo"),
                "domain_id": LaunchConfiguration("domain_id"),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
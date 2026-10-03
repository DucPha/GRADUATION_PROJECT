from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("demo", default_value="false"),
        DeclareLaunchArgument("domain_id", default_value="0"),
    ]

    demo_cfg = LaunchConfiguration("demo")
    domain_cfg = LaunchConfiguration("domain_id")

    node = Node(
        package="autonomous_vehicle_gui",
        executable="autonomous_vehicle_gui_node",
        name="autonomous_vehicle_gui_node",
        output="screen",
        parameters=[
            {
                "demo": PythonExpression(["'true' if ", demo_cfg, " else 'false'"]),
                "domain_id": PythonExpression([domain_cfg]),
            }
        ],
    )

    return LaunchDescription(arguments + [node])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_gui_arg = DeclareLaunchArgument(
        'enable_gui',
        default_value='false',
        description='Bật/tắt GUI Python + Matplotlib'
    )

    gui_node = Node(
        package='gui_matplotlib',
        executable='gui_matplotlib',
        name='gui_matplotlib',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gui'))
    )

    return LaunchDescription([
        enable_gui_arg,
        gui_node,
    ])

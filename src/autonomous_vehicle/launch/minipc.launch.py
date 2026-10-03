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
        DeclareLaunchArgument("camera_fps", default_value="30"),
        # Đường điều khiển tách khỏi ảnh hiển thị. control_hz là tần số gửi
        # lệnh ESP32, không được phụ thuộc tốc độ encode/publish ảnh.
        DeclareLaunchArgument("control_hz", default_value="100"),
        # Ảnh 1265x900. Ở 120 Hz là ~126 MB/s trên DDS, gần hết băng thông
        # card mạng 100Mbps. 10 Hz là đủ để người ngồi xe/debug quan sát.
        DeclareLaunchArgument("viz_hz", default_value="10"),
        DeclareLaunchArgument("enable_viz", default_value="true"),
        # ROI: tỉ lệ chiều cao ảnh. Mép trên = tầm xa, mép dưới = gần nhất.
        # Đáy ảnh ở đây là nền cách bumper 30-40cm. Giảm số này để lấy ROI
        # cao hơn (xa hơn), tăng để chỉ lấy dải gần.
        DeclareLaunchArgument("roi_factor_dual", default_value="0.70"),
        DeclareLaunchArgument("roi_factor_single", default_value="0.85"),
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
                    "camera_fps": ParameterValue(
                        LaunchConfiguration("camera_fps"), value_type=int
                    ),
                    "control_hz": ParameterValue(
                        LaunchConfiguration("control_hz"), value_type=float
                    ),
                    "viz_hz": ParameterValue(
                        LaunchConfiguration("viz_hz"), value_type=float
                    ),
                    "enable_viz": ParameterValue(
                        LaunchConfiguration("enable_viz"), value_type=bool
                    ),
                    "roi_factor_dual": ParameterValue(
                        LaunchConfiguration("roi_factor_dual"), value_type=float
                    ),
                    "roi_factor_single": ParameterValue(
                        LaunchConfiguration("roi_factor_single"), value_type=float
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
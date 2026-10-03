"""System bringup launch cho Autonomous Vehicle (nguồn tham số duy nhất).

Chạy toàn bộ hệ thống:
  * C++ fusion_viz_node : camera + LiDAR + obstacle avoidance + UART ESP32
  * traffic_light_detector : nhận dạng đèn giao thông / biển (NCNN)
  * turn_detector         : nhận dạng mũi tên rẽ (NCNN)
  * rviz2 (tuỳ chọn)

Đây là file launch DUY NHẤT chứa danh sách node. `minipc.launch.py` chỉ là
wrapper gọi lại file này, tránh tình trạng hai bản lệch tham số với nhau.

Lưu ý quan trọng về đồng bộ biến môi trường:
- Model paths dùng biến môi trường TRAFFIC_MODEL_DIR, TURN_MODEL_DIR
- Fallback về relative paths trong package share directory
- KHÔNG dùng hardcoded ~/yolo_ws hay ~/models (không portable)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _default_traffic_model(name):
    """Trả về đường dẫn model traffic light.
    
    Ưu tiên:
    1. Biến môi trường TRAFFIC_MODEL_DIR
    2. Thư mục share của package traffic_light_detector
    3. Hardcoded path cũ (chỉ để tương thích ngược)
    """
    # 1. Thử biến môi trường
    env_dir = os.environ.get("TRAFFIC_MODEL_DIR")
    if env_dir:
        return os.path.join(os.path.expandvars(os.path.expanduser(env_dir)), name)
    
    # 2. Thử package share directory
    try:
        pkg_share = get_package_share_directory("traffic_light_detector")
        model_path = os.path.join(pkg_share, "models", name)
        if os.path.exists(model_path):
            return model_path
    except Exception:
        pass
    
    # 3. Fallback về path cũ (chỉ để không break code hiện có)
    return os.path.expanduser(
        f"~/yolo_ws/runs/detect/traffic_all_red_turn_e10/weights/best_ncnn_model/{name}"
    )


def _default_turn_model(name):
    """Trả về đường dẫn model turn detector.
    
    Ưu tiên:
    1. Biến môi trường TURN_MODEL_DIR
    2. Thư mục share của package turn_detector
    3. Hardcoded path cũ (chỉ để tương thích ngược)
    """
    # 1. Thử biến môi trường
    env_dir = os.environ.get("TURN_MODEL_DIR")
    if env_dir:
        return os.path.join(os.path.expandvars(os.path.expanduser(env_dir)), name)
    
    # 2. Thử package share directory
    try:
        pkg_share = get_package_share_directory("turn_detector")
        model_path = os.path.join(pkg_share, "models", name)
        if os.path.exists(model_path):
            return model_path
    except Exception:
        pass
    
    # 3. Fallback về path cũ
    return os.path.expanduser(f"~/models/turn_lr_ncnn_model/{name}")


def generate_launch_description():
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    args = [
        DeclareLaunchArgument("namespace", default_value="",
                              description="Namespace cho toàn bộ node"),
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Dùng đồng hồ mô phỏng"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="Mức log của các node"),

        # Hardware
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0",
                              description="Cổng serial ESP32 (thử cả /dev/ttyACM0 nếu USB0 fail)"),
        DeclareLaunchArgument("baudrate", default_value="230400",
                              description="Baudrate UART (phải khớp firmware)"),
        DeclareLaunchArgument(
            "camera_index", default_value="-1",
            description=(
                "Chỉ số V4L2 của camera. -1 = tự dò /dev/video*. Bắt buộc "
                "dùng -1 với camera USB này: khi rớt khỏi bus USB rồi cắm "
                "lại, kernel cấp số /dev/video khác (video0 <-> video1)"
            )),
        DeclareLaunchArgument(
            "camera_fps", default_value="30",
            description=(
                "FPS yêu cầu. Camera đo được 63 FPS ở 352x288 (độ phân giải "
                "mặc định trong camera_lane.hpp). 640x480 chỉ đạt ~6 FPS vì "
                "vượt băng thông USB 2.0"
            )),
        DeclareLaunchArgument(
            "camera_auto_exposure", default_value="true",
            description=(
                "Auto-exposure. Camera này KHÔNG có control exposure_auto / "
                "exposure_absolute nên giá trị này không có tác dụng thực tế; "
                "giữ true cho khớp driver"
            )),
        DeclareLaunchArgument("scan_topic", default_value="/scan",
                              description="Topic LaserScan của LiDAR"),
        DeclareLaunchArgument("lidar_mount_offset_deg", default_value="-90.0",
                              description=(
                                  "Góc lệch lắp đặt LiDAR theo khung xe (REP-103): "
                                  "0=trước, 90=trái, 180=sau, 270=phải"
                              )),

        # Visualization
        # Ảnh HUD 1000x730 ở 120 Hz là ~126 MB/s trên DDS, gần bằng hết băng
        # thông một card mạng 100 Mbps. Node cũng đã tách control (100 Hz)
        # khỏi viz, nên viz chỉ cần đủ mượt để người vận hành nhìn.
        DeclareLaunchArgument("viz_hz", default_value="10.0",
                              description="Tần số vẽ HUD (Hz)"),
        DeclareLaunchArgument("image_topic", default_value="/fusion_viz/image",
                              description="Topic ảnh HUD có vẽ bản đồ"),
        DeclareLaunchArgument("raw_image_topic", default_value="/image_raw",
                              description="Topic ảnh thô cho node AI"),
        DeclareLaunchArgument("raw_image_hz", default_value="5.0",
                              description="Tần số phát ảnh thô (Hz)"),
        DeclareLaunchArgument("enable_rviz", default_value="false",
                              description="Khởi chạy RViz2"),

        # Topic AI. Khai ở đây để node C++ và node Python cùng đọc một nguồn:
        # trước đây mỗi bên tự viết tên topic trong code, nên đổi tên ở một bên
        # làm hai bên không nói chuyện với nhau mà không có cảnh báo.
        DeclareLaunchArgument("traffic_light_decision_topic",
                              default_value="/traffic_light/decision"),
        DeclareLaunchArgument("turn_detector_decision_topic",
                              default_value="/turn_detector/decision"),
        DeclareLaunchArgument("traffic_debug_image_topic",
                              default_value="/traffic_light/image_debug"),
        DeclareLaunchArgument("turn_debug_image_topic",
                              default_value="/turn_detector/image_debug"),

        # Lách tay theo biển rẽ (xử lý ở fusion node)
        DeclareLaunchArgument("turn_blend_px", default_value="30",
                              description="Góc lái bù khi nhận biển rẽ (px, dương=phải)"),
        DeclareLaunchArgument("turn_speed_x10", default_value="40",
                              description="Trần tốc độ khi nhận biển rẽ (km/h x 10)"),
        DeclareLaunchArgument(
            "speed_normal_x10", default_value="0",
            description=(
                "Tốc độ ở state NORMAL (km/h x 10). 0 = để bộ lập của "
                "detector quyết: thẳng 85 / cua 60 / cua gấp 45. Đặt 35 để "
                "quay lại hành vi cũ (đường thẳng cố định 3.5 km/h)"
            )),

        # Watchdog (giá trị cũ hơn = phanh sớm hơn)
        DeclareLaunchArgument("camera_stale_timeout_s", default_value="1.0",
                              description="Ngưỡng coi camera là đứng hình (s)"),
        DeclareLaunchArgument("lidar_stale_timeout_s", default_value="0.5",
                              description="Ngưỡng coi LiDAR là mất dữ liệu (s)"),
        DeclareLaunchArgument("ai_timeout_s", default_value="2.0",
                              description="Ngưỡng coi quyết định AI là cũ (s)"),

        # AI detectors
        DeclareLaunchArgument("enable_traffic", default_value="true",
                              description="Bật node nhận dạng đèn giao thông"),
        DeclareLaunchArgument("enable_turn", default_value="true",
                              description="Bật node nhận dạng mũi tên rẽ"),
        DeclareLaunchArgument("traffic_model_param",
                              default_value=_default_traffic_model("model.ncnn.param")),
        DeclareLaunchArgument("traffic_model_bin",
                              default_value=_default_traffic_model("model.ncnn.bin")),
        DeclareLaunchArgument("turn_model_param",
                              default_value=_default_turn_model("model.ncnn.param")),
        DeclareLaunchArgument("turn_model_bin",
                              default_value=_default_turn_model("model.ncnn.bin")),
        DeclareLaunchArgument("traffic_infer_hz", default_value="30.0",
                              description="Tần số infer của node đèn giao thông"),
        DeclareLaunchArgument("turn_infer_hz", default_value="30.0",
                              description="Tần số infer của node mũi tên rẽ"),
    ]

    # =========================================================================
    # Node C++ chính
    # =========================================================================
    # Mọi tham số số phải ép kiểu rõ ràng. LaunchConfiguration trả về chuỗi;
    # nếu không ép, node C++ nhận string và declare_parameter("camera_fps", 30)
    # sẽ ném exception ParameterTypeException, node chết ngay lúc khởi động.
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
            "camera_auto_exposure": ParameterValue(
                LaunchConfiguration("camera_auto_exposure"), value_type=bool),
            "viz_hz": ParameterValue(LaunchConfiguration("viz_hz"), value_type=float),
            "image_topic": LaunchConfiguration("image_topic"),
            "raw_image_topic": LaunchConfiguration("raw_image_topic"),
            "raw_image_hz": ParameterValue(LaunchConfiguration("raw_image_hz"),
                                           value_type=float),
            "lidar_mount_offset_deg": ParameterValue(
                LaunchConfiguration("lidar_mount_offset_deg"), value_type=float),
            "camera_stale_timeout_s": ParameterValue(
                LaunchConfiguration("camera_stale_timeout_s"), value_type=float),
            "lidar_stale_timeout_s": ParameterValue(
                LaunchConfiguration("lidar_stale_timeout_s"), value_type=float),
            "ai_timeout_s": ParameterValue(LaunchConfiguration("ai_timeout_s"),
                                           value_type=float),
            "traffic_light_topic": LaunchConfiguration("traffic_light_decision_topic"),
            "turn_detector_topic": LaunchConfiguration("turn_detector_decision_topic"),
            "turn_blend_px": ParameterValue(
                LaunchConfiguration("turn_blend_px"), value_type=int),
            "turn_speed_x10": ParameterValue(
                LaunchConfiguration("turn_speed_x10"), value_type=int),
            "speed_normal_x10": ParameterValue(
                LaunchConfiguration("speed_normal_x10"), value_type=int),
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
            "infer_hz": ParameterValue(LaunchConfiguration("traffic_infer_hz"),
                                       value_type=float),
            # Cùng tên topic với fusion node => không thể lệch âm thầm.
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
            "infer_hz": ParameterValue(LaunchConfiguration("turn_infer_hz"),
                                       value_type=float),
            "image_topic": LaunchConfiguration("raw_image_topic"),
            "decision_topic": LaunchConfiguration("turn_detector_decision_topic"),
            "debug_image_topic": LaunchConfiguration("turn_debug_image_topic"),
        }],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        respawn=True,
        respawn_delay=3.0,
    )

    # RViz2 dùng đường dẫn config trong share directory, không hard-code
    # /home/<user>/Documents/... (bản cũ chỉ chạy được trên máy tác giả).
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_rviz")),
        arguments=["-d", os.path.join(
            get_package_share_directory("autonomous_vehicle"), "rviz", "autonomous.rviz")],
    )

    # =========================================================================
    # Trình tự khởi động
    # =========================================================================
    nodes = [
        # Node C++ chạy trước: nó là nguồn phát /image_raw cho các node AI.
        cpp_node,

        # Node AI khởi động sau 2s để /image_raw đã có dữ liệu.
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
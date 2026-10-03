#!/usr/bin/env python3
"""Launch duy nhat cua he thong phase 1.

Tu do cong serial cho ESP32 (USB-OTG, /dev/ttyACM*) va cho LiDAR
(/dev/ttyUSB* qua USB-UART), nen khong can truyen cong tay. Van co the ghi
de: serial_port:= / lidar_port:= ...
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ESP_HINTS = ('esp32', 'espressif', 'silabs', 'esp_')
LIDAR_HINTS = ('rplidar', 'sls', 'ch340', 'usb-serial', 'cp210', 'ft232', 'ftdi')


def _all_devices():
    devices = []
    for pattern in ('/dev/serial/by-id/*', '/dev/ttyUSB*', '/dev/ttyACM*'):
        devices.extend(sorted(glob.glob(pattern)))
    return devices


def detect_esp_port():
    """ESP32-S3 qua USB-OTG: uu tien ten by-id, sau do ttyACM*, cuoi cung ttyUSB*."""
    devices = _all_devices()

    for dev in devices:
        name = os.path.basename(dev).lower()
        if '/dev/serial/by-id/' in dev and any(h in name for h in ESP_HINTS):
            return dev

    for dev in devices:
        if dev.startswith('/dev/ttyACM'):
            return dev

    for dev in devices:
        if dev.startswith('/dev/ttyUSB'):
            return dev

    return ''


def detect_lidar_port(esp_port):
    """LiDAR la USB-UART nen la ttyUSB*, va phai khac cong cua ESP32."""
    esp_real = os.path.realpath(esp_port) if esp_port else ''
    devices = [
        d for d in _all_devices()
        if not esp_real or os.path.realpath(d) != esp_real
    ]

    for dev in devices:
        name = os.path.basename(dev).lower()
        if '/dev/serial/by-id/' in dev and any(h in name for h in LIDAR_HINTS):
            return dev

    for dev in devices:
        if dev.startswith('/dev/ttyUSB'):
            return dev

    for dev in devices:
        if dev.startswith('/dev/ttyACM'):
            return dev

    return ''


def is_true(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def spawn_lidar(context, *args, **kwargs):
    """Driver LiDAR. Cong lay tu tham so tai launch, mac dinh la cong tu do."""
    if not is_true(LaunchConfiguration('enable_lidar').perform(context)):
        return [LogInfo(msg=['[fusion] enable_lidar:=false, bo qua driver LiDAR'])]

    port = LaunchConfiguration('lidar_port').perform(context)

    if not port:
        return [LogInfo(msg=[
            '[fusion] Khong tim thay cong USB thu hai cho LiDAR. '
            'Xe van chay, chi khong co /scan.'
        ])]

    return [ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rplidar_ros', 'rplidar_composition',
            '--ros-args',
            '-p', 'serial_port:=%s' % port,
            '-p', 'serial_baudrate:=115200',
            '-p', 'frame_id:=laser',
            '-p', 'inverted:=false',
            '-p', 'angle_compensate:=true',
        ],
        output='screen',
    )]


ESP_PORT = detect_esp_port()
LIDAR_PORT = detect_lidar_port(ESP_PORT)


def generate_launch_description():

    node_args = {
        'serial_port': LaunchConfiguration('serial_port'),
        'camera_index': LaunchConfiguration('camera_index'),
        'camera_fps': LaunchConfiguration('camera_fps'),
        'roi_top_frac': LaunchConfiguration('roi_top_frac'),
        'speed_x10': LaunchConfiguration('speed_x10'),
        'speed_hold_x10': LaunchConfiguration('speed_hold_x10'),
        'dev_sign': LaunchConfiguration('dev_sign'),
        'lane_lost_stop_ms': LaunchConfiguration('lane_lost_stop_ms'),
        'control_hz': LaunchConfiguration('control_hz'),
        'viz_hz': LaunchConfiguration('viz_hz'),
        'enable_viz': LaunchConfiguration('enable_viz'),
        'enable_lidar': LaunchConfiguration('enable_lidar'),
        'lidar_mount_offset_deg': LaunchConfiguration('lidar_mount_offset_deg'),
        'log_level': LaunchConfiguration('log_level'),
    }

    declare = [
        DeclareLaunchArgument('serial_port', default_value='',
                              description='Cong ESP32. De trong = tu do.'),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT,
                              description='Cong LiDAR. Mac dinh = tu do.'),
        DeclareLaunchArgument('camera_index', default_value='-1',
                              description='Chi so camera, -1 = tu do 0..3'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('roi_top_frac', default_value='0.45',
                              description='0.0-1.0, phan tram tren cua anh'),
        DeclareLaunchArgument('speed_x10', default_value='40',
                              description='Toc do khi co 2 vanh, km/h x 10'),
        DeclareLaunchArgument('speed_hold_x10', default_value='20',
                              description='Toc do khi mat 2 vanh, km/h x 10'),
        DeclareLaunchArgument('dev_sign', default_value='1',
                              description='-1 neu servo lap nguoc'),
        DeclareLaunchArgument('lane_lost_stop_ms', default_value='400'),
        DeclareLaunchArgument('control_hz', default_value='100'),
        DeclareLaunchArgument('viz_hz', default_value='10'),
        DeclareLaunchArgument('enable_viz', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_mount_offset_deg', default_value='-90.0',
                              description='Goc lech lap LiDAR, 0=truoc 90=trai (REP-103)'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    log = [
        LogInfo(msg=['[fusion] ESP32 port (auto): %s' % (ESP_PORT or 'KHONG TIM THAY')]),
        LogInfo(msg=['[fusion] LiDAR port (auto): %s' % (LIDAR_PORT or 'KHONG TIM THAY')]),
    ]

    fusion = Node(
        package='fusion_node',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[node_args],
    )

    return LaunchDescription(
        declare + log + [OpaqueFunction(function=spawn_lidar), fusion]
    )
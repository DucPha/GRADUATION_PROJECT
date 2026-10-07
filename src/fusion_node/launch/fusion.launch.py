#!/usr/bin/env python3
"""Launch duy nhat cua he thong phase 1.

Phan cung tren xe (giao tiep UART qua chip CP2102, KHONG dung USB-OTG):
    ESP32  -> /dev/ttyUSB0  (230400 baud, protocol nhi phan 11/7 byte)
    LiDAR  -> /dev/ttyUSB1  (RPLIDAR A1, 115200 baud)
    Camera -> /dev/video*   (MJPG 1920x1080 @ 30 fps)

Ca 2 cong deu la CP2102 cung so serial nen ten /dev/serial/by-id khong phan
biet duoc. Neu cong mac dinh khong ton tai (doi thu tu cam USB), launch tu do:
cong nao gui goi telemetry 0xDC 0xBA cua ESP32 la ESP32, cong con lai la LiDAR.
Van ghi de duoc: serial_port:=/dev/ttyUSB1 lidar_port:=/dev/ttyUSB0
"""

import glob
import os
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_ESP_PORT = '/dev/ttyUSB0'
DEFAULT_LIDAR_PORT = '/dev/ttyUSB1'
ESP_BAUD = 230400


def _serial_ports():
    return sorted(glob.glob('/dev/ttyUSB*')) + sorted(glob.glob('/dev/ttyACM*'))


def _same(a, b):
    return bool(a) and bool(b) and os.path.realpath(a) == os.path.realpath(b)


def _probe_esp32(port, timeout_s=0.7):
    """Nghe cong `port` xem co goi telemetry cua ESP32 khong (DC BA 7 byte hoac
    DC BB 10 byte, XOR cac byte giua).

    Chi doc, khong gui gi. DTR/RTS ha xuong de ESP32 khong bi reset.
    """
    try:
        import serial  # python3-serial
    except ImportError:
        return False
    try:
        s = serial.Serial()
        s.port, s.baudrate, s.timeout = port, ESP_BAUD, 0.05
        s.dtr = False
        s.rts = False
        s.open()
    except Exception:
        return False
    buf = b''
    t_end = time.time() + timeout_s
    try:
        while time.time() < t_end:
            buf += s.read(256)
            for hdr, n in ((b'\xdc\xba', 7), (b'\xdc\xbb', 10)):
                i = buf.find(hdr)
                while 0 <= i <= len(buf) - n:
                    p = buf[i:i + n]
                    x = 0
                    for c in p[2:n - 1]:
                        x ^= c
                    if x == p[n - 1]:
                        return True
                    i = buf.find(hdr, i + 1)
    finally:
        s.close()
    return False


def resolve_ports(esp_req, lidar_req):
    """Tra ve (esp_port, lidar_port, ghi_chu). Uu tien cong nguoi dung chon."""
    note = []
    esp = esp_req if os.path.exists(esp_req) else ''
    lidar = lidar_req if os.path.exists(lidar_req) else ''

    if not esp:
        note.append('%s khong ton tai, dang do ESP32...' % esp_req)
        for p in _serial_ports():
            if not _same(p, lidar) and _probe_esp32(p):
                esp = p
                break
    if not lidar:
        note.append('%s khong ton tai, lay cong USB con lai cho LiDAR' % lidar_req)
        for p in _serial_ports():
            if not _same(p, esp):
                lidar = p
                break
    if _same(esp, lidar):
        note.append('ESP32 va LiDAR trung cong %s -> tat LiDAR' % esp)
        lidar = ''
    return esp, lidar, note


def is_true(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def setup(context, *args, **kwargs):
    esp, lidar, note = resolve_ports(
        LaunchConfiguration('serial_port').perform(context),
        LaunchConfiguration('lidar_port').perform(context))

    actions = [LogInfo(msg='[fusion] ' + n) for n in note]
    actions.append(LogInfo(msg='[fusion] ESP32 (UART): %s' % (esp or 'KHONG TIM THAY')))
    actions.append(LogInfo(msg='[fusion] LiDAR       : %s' % (lidar or 'KHONG TIM THAY')))

    enable_lidar = is_true(LaunchConfiguration('enable_lidar').perform(context))
    if enable_lidar and lidar:
        actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rplidar_ros', 'rplidar_composition',
                '--ros-args',
                '-p', 'serial_port:=%s' % lidar,
                '-p', 'serial_baudrate:=115200',
                '-p', 'frame_id:=laser',
                '-p', 'inverted:=false',
                '-p', 'angle_compensate:=true',
            ],
            output='screen',
        ))
    elif enable_lidar:
        actions.append(LogInfo(msg='[fusion] Khong co cong LiDAR: xe van chay, chi khong co /scan.'))

    def arg(name):
        return LaunchConfiguration(name)

    params = {
        'serial_port': esp or LaunchConfiguration('serial_port').perform(context),
        'lidar_port': lidar,
        'camera_index': arg('camera_index'),
        'camera_fps': arg('camera_fps'),
        'camera_width': arg('camera_width'),
        'camera_height': arg('camera_height'),
        'camera_exposure': arg('camera_exposure'),
        'horizon_frac': arg('horizon_frac'),
        'camera_height_m': arg('camera_height_m'),
        'speed_min_x10': arg('speed_min_x10'),
        'roi_top_frac': arg('roi_top_frac'),
        'speed_x10': arg('speed_x10'),
        'speed_hold_x10': arg('speed_hold_x10'),
        'speed_corner_x10': arg('speed_corner_x10'),
        'speed_ramp_x10': arg('speed_ramp_x10'),
        'dev_sign': arg('dev_sign'),
        'lane_lost_stop_ms': arg('lane_lost_stop_ms'),
        'control_hz': arg('control_hz'),
        'viz_hz': arg('viz_hz'),
        'status_hz': arg('status_hz'),
        'enable_viz': arg('enable_viz'),
        'enable_lidar': arg('enable_lidar'),
        'lidar_mount_offset_deg': arg('lidar_mount_offset_deg'),
        'require_start': arg('require_start'),
        'start_timeout_ms': arg('start_timeout_ms'),
        'log_level': arg('log_level'),
    }

    actions.append(Node(
        package='fusion_node',
        executable='fusion_node',
        name='fusion_node',
        output='screen',
        parameters=[params],
        # Camera nay thinh thoang khong tra frame dau tien sau khi mo; OpenCV mac
        # dinh doi 10 s moi bao loi. Ha xuong 2 s de CameraLane thu mo lai som.
        additional_env={'OPENCV_VIDEOIO_V4L_SELECT_TIMEOUT': '2'},
    ))
    return actions


def generate_launch_description():
    declare = [
        DeclareLaunchArgument('serial_port', default_value=DEFAULT_ESP_PORT,
                              description='Cong UART cua ESP32'),
        DeclareLaunchArgument('lidar_port', default_value=DEFAULT_LIDAR_PORT,
                              description='Cong UART cua LiDAR'),
        DeclareLaunchArgument('camera_index', default_value='-1',
                              description='Chi so camera, -1 = tu do 0..3'),
        DeclareLaunchArgument('camera_fps', default_value='30'),
        DeclareLaunchArgument('camera_width', default_value='1920'),
        DeclareLaunchArgument('camera_height', default_value='1080'),
        DeclareLaunchArgument('camera_exposure', default_value='-1',
                              description='-1 = tu dong; >0 = phoi sang tay (100 us), vd 250 giu 30 fps'),
        DeclareLaunchArgument('horizon_frac', default_value='0.20',
                              description='Chan troi (ti le chieu cao anh): duong tim tren overlay '
                                          'phai di qua diem 2 vach thang keo dai gap nhau'),
        DeclareLaunchArgument('camera_height_m', default_value='0.30',
                              description='Do cao camera so voi mat duong (m)'),
        DeclareLaunchArgument('speed_min_x10', default_value='25',
                              description='Toc do nho nhat khi chay (km/h x10); banh khong quay thi tang'),
        DeclareLaunchArgument('roi_top_frac', default_value='0.52',
                              description='0.0-1.0, dinh ROI tinh tu tren; nho hon = thay xa hon'),
        DeclareLaunchArgument('speed_x10', default_value='30',
                              description='Toc do duong thang (2 vach), km/h x 10'),
        DeclareLaunchArgument('speed_hold_x10', default_value='15',
                              description='Toc do giu huong khi mat ca 2 vach, km/h x 10'),
        DeclareLaunchArgument('speed_corner_x10', default_value='15',
                              description='Toc do khi chi thay 1 vach (khuc cua), km/h x 10'),
        DeclareLaunchArgument('speed_ramp_x10', default_value='8',
                              description='Muc tang toc, x10 moi giay'),
        DeclareLaunchArgument('dev_sign', default_value='1',
                              description='-1 neu servo lap nguoc'),
        DeclareLaunchArgument('lane_lost_stop_ms', default_value='400'),
        DeclareLaunchArgument('control_hz', default_value='100'),
        DeclareLaunchArgument('viz_hz', default_value='30',
                              description='Tan so gui anh cho GUI (chi gui khi co nguoi xem)'),
        DeclareLaunchArgument('status_hz', default_value='10'),
        DeclareLaunchArgument('enable_viz', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_mount_offset_deg', default_value='-90.0',
                              description='Goc lech lap LiDAR, 0=truoc 90=trai (REP-103)'),
        DeclareLaunchArgument('require_start', default_value='true',
                              description='true = xe chi chay khi bam SPACE tren GUI'),
        DeclareLaunchArgument('start_timeout_ms', default_value='600',
                              description='Mat heartbeat /autocar/run qua muc nay -> dung xe'),
        DeclareLaunchArgument('log_level', default_value='info'),
    ]

    return LaunchDescription(declare + [OpaqueFunction(function=setup)])

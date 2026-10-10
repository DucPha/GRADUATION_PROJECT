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
from launch_ros.parameter_descriptions import ParameterValue

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
        'camera_height_m': arg('camera_height_m'),
        'camera_pitch_deg': arg('camera_pitch_deg'),
        'camera_vfov_deg': arg('camera_vfov_deg'),
        'speed_min_x10': arg('speed_min_x10'),
        'roi_top_frac': arg('roi_top_frac'),
        'speed_x10': arg('speed_x10'),
        'speed_hold_x10': arg('speed_hold_x10'),
        'speed_corner_x10': arg('speed_corner_x10'),
        'speed_down_x10': arg('speed_down_x10'),
        'speed_one_x10': arg('speed_one_x10'),
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
        'wheelbase_m': arg('wheelbase_m'),
        'cam_to_rear_axle_m': arg('cam_to_rear_axle_m'),
        'steer_ratio': arg('steer_ratio'),
        'lookahead_min_m': arg('lookahead_min_m'),
        'lookahead_max_m': arg('lookahead_max_m'),
        'lookahead_gain_s': arg('lookahead_gain_s'),
        'lookahead_corner_scale': arg('lookahead_corner_scale'),
        'camera_latency_s': arg('camera_latency_s'),
        'actuator_latency_s': arg('actuator_latency_s'),
        'car_half_width_m': arg('car_half_width_m'),
        'line_margin_m': arg('line_margin_m'),
        'odom_speed_scale': arg('odom_speed_scale'),
        'lost_memory_ms': arg('lost_memory_ms'),
        'steer_filter_s': arg('steer_filter_s'),
        'steer_rate_dps': ParameterValue(arg('steer_rate_dps'), value_type=float),
        'steer_rate_corner_dps': ParameterValue(arg('steer_rate_corner_dps'), value_type=float),
        'tape_half_m': arg('tape_half_m'),
        'corner_gain': arg('corner_gain'),
        'coast_decel_mps2': arg('coast_decel_mps2'),
        'corner_margin_m': arg('corner_margin_m'),
        'corner_outer_gap_m': arg('corner_outer_gap_m'),
        'corner_sharp_k': arg('corner_sharp_k'),
        'kink_radius_m': arg('kink_radius_m'),
        'corner_outer_max_m': arg('corner_outer_max_m'),
        'lane_start_frames': arg('lane_start_frames'),
        'xte_gain': arg('xte_gain'),
        'single_search_m': arg('single_search_m'),
        'record_frames': arg('record_frames'),
        'record_dir': arg('record_dir'),
        'single_search_rate': arg('single_search_rate'),
        'xte_ki': arg('xte_ki'),
        'xte_i_max_deg': arg('xte_i_max_deg'),
        'xte_far_m': arg('xte_far_m'),
        'xte_far_gain': arg('xte_far_gain'),
        'pid_kp': ParameterValue(arg('pid_kp'), value_type=float),
        'pid_kd': ParameterValue(arg('pid_kd'), value_type=float),
        'pid_ki': ParameterValue(arg('pid_ki'), value_type=float),
        'pid_i_max_deg': ParameterValue(arg('pid_i_max_deg'), value_type=float),
        'pid_max_deg': ParameterValue(arg('pid_max_deg'), value_type=float),
        'pid_filter_s': ParameterValue(arg('pid_filter_s'), value_type=float),
        # node khai bao double: '90' (int) lam node chet ngay khi khoi dong
        'servo_straight_deg': ParameterValue(arg('servo_straight_deg'), value_type=float),
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
        DeclareLaunchArgument('camera_exposure', default_value='0',
                              description='0 = tu do do sang san ~110 roi KHOA (giu 30 fps, khong bi '
                                          'den tran lam nhay sang toi); >0 = co dinh (100 us); -1 = camera tu dong'),
        DeclareLaunchArgument('camera_height_m', default_value='0.30',
                              description='Do cao tam ong kinh so voi mat san (m)'),
        DeclareLaunchArgument('camera_pitch_deg', default_value='35.5',
                              description='Goc cui camera (do), co dinh. 08/10 ~42; toi 09/10 camera ngua len: 35.5 '
                                          '(do bang diem tu 2 dai thang song song; doi goc camera thi do lai)'),
        DeclareLaunchArgument('camera_vfov_deg', default_value='51.0',
                              description='Goc nhin DOC cua camera (do), giong nhau o 1080p va 640x480'),
        DeclareLaunchArgument('speed_min_x10', default_value='58',
                              description='Toc do lenh nho nhat khi chay (km/h x10) = toc do cua gat. 58 = ~1557.5 us (+3 us khi het lai; 52 van keu). '
                                          'Toc do THAT chi ~0.4 m/s (do 09/10) va giam khi pin yeu -> BLDC keu thi tang'),
        DeclareLaunchArgument('roi_top_frac', default_value='0.0',
                              description='0.0-1.0, phan anh tren cung bo qua; camera cui ~42 do nen ca '
                                          'anh deu la mat san (~0.13-1.1 m), mac dinh dung het'),
        DeclareLaunchArgument('speed_x10', default_value='70',
                              description='Toc do du 2 vach + duong thang + xe giua lan, km/h x 10 (70 = ~1565 us, '
                                          '65 = ~1562 us ban truoc 10/10 18:05)'),
        DeclareLaunchArgument('speed_hold_x10', default_value='58',
                              description='Toc do giu huong khi mat ca 2 vach, km/h x 10'),
        DeclareLaunchArgument('speed_corner_x10', default_value='58',
                              description='Toc do vao cua (1 vach, hoac 2 vach cong gat nhat), km/h x 10. '
                                          '58 = ~1557.5 us (firmware san 1550 us); moi don vi ~0.65 us'),
        DeclareLaunchArgument('speed_down_x10', default_value='20',
                              description='Giam toc khi vao cua (km/h x10 moi giay): nhanh nhung deu, khong giat'),
        DeclareLaunchArgument('speed_one_x10', default_value='60',
                              description='Toc do chi thay 1 vach tren duong thang + xe giua lan, km/h x 10 '
                                          '(60 = ~1559 us); cua / xe lech -> speed_corner_x10'),
        DeclareLaunchArgument('speed_ramp_x10', default_value='12',
                              description='Muc tang toc khi ra khoi cua, x10 moi giay (12: 58 -> 70 deu trong ~1 s; '
                                          '30 cu = gan nhu nhay bac)'),
        DeclareLaunchArgument('dev_sign', default_value='1',
                              description='-1 neu servo lap nguoc'),
        DeclareLaunchArgument('lane_lost_stop_ms', default_value='2500',
                              description='Mat ca 2 vach lau hon muc nay moi dung; thay lai lan thi tu chay'),
        DeclareLaunchArgument('control_hz', default_value='100'),
        DeclareLaunchArgument('viz_hz', default_value='30',
                              description='Tan so gui anh cho GUI (chi gui khi co nguoi xem)'),
        DeclareLaunchArgument('status_hz', default_value='10'),
        DeclareLaunchArgument('enable_viz', default_value='true'),
        DeclareLaunchArgument('enable_lidar', default_value='true'),
        DeclareLaunchArgument('lidar_mount_offset_deg', default_value='90.0',
                              description='Goc lech lap LiDAR (do): goc_xe = goc_scan + offset, '
                                          '0=truoc 90=trai (REP-103). +90 = goc 0 tho cua RPLIDAR '
                                          'chia sang BEN PHAI xe'),
        DeclareLaunchArgument('require_start', default_value='true',
                              description='true = xe chi chay khi bam SPACE tren GUI'),
        DeclareLaunchArgument('start_timeout_ms', default_value='600',
                              description='Mat heartbeat /autocar/run qua muc nay -> dung xe'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # ---- Lai theo duong da nho (PathTracker) - DO 3 SO DAU TREN XE THAT ----
        DeclareLaunchArgument('wheelbase_m', default_value='0.26',
                              description='Khoang cach truc truoc - truc sau (m)'),
        DeclareLaunchArgument('cam_to_rear_axle_m', default_value='0.18',
                              description='Khoang cach doc tu TRUC SAU toi diem ngay duoi camera (m). '
                                          'ANH HUONG LON toi bam lan. Do tren xe: ~0.18'),
        DeclareLaunchArgument('steer_ratio', default_value='0.6',
                              description='Goc banh / goc servo (servo lech 30 do ma banh lech 24 do -> 0.8)'),
        DeclareLaunchArgument('lookahead_min_m', default_value='0.35',
                              description='Khoang nhin truoc pure pursuit = min + gain * v (m)'),
        DeclareLaunchArgument('lookahead_max_m', default_value='0.80'),
        DeclareLaunchArgument('lookahead_gain_s', default_value='0.20'),
        DeclareLaunchArgument('lookahead_corner_scale', default_value='0.8',
                              description='Trong cua nhin GAN hon: nhan khoang nhin truoc voi he so nay (diem ngam '
                                          'khong roi len doan thang sau cua); 1 = tat'),
        DeclareLaunchArgument('camera_latency_s', default_value='0.04',
                              description='Tre tu luc chup toi luc co ket qua detector (s)'),
        DeclareLaunchArgument('actuator_latency_s', default_value='0.08',
                              description='Tre servo (s)'),
        DeclareLaunchArgument('car_half_width_m', default_value='0.125',
                              description='Nua be ngang xe (m), xe rong 25 cm; dung cho rao chan vach'),
        DeclareLaunchArgument('tape_half_m', default_value='0.035',
                              description='Nua be rong bang keo vach (m), bang keo 7 cm'),
        DeclareLaunchArgument('line_margin_m', default_value='0.02',
                              description='Khoang trong toi thieu tu than xe toi MEP bang keo (m)'),
        DeclareLaunchArgument('corner_gain', default_value='1.15',
                              description='Trong cua nhan goc lai pure pursuit voi he so nay (1 = tat)'),
        DeclareLaunchArgument('coast_decel_mps2', default_value='0.45',
                              description='Gia toc giam khi nha ga (m/s^2): thay cua cach d m -> toc do '
                                          'sqrt(v_cua^2 + 2*a*(d - corner_margin_m)). Van vot cua -> giam'),
        DeclareLaunchArgument('corner_margin_m', default_value='0.20',
                              description='Phai xuong toc do cua truoc khi toi cua bay nhieu m'),
        DeclareLaunchArgument('corner_sharp_k', default_value='1.0',
                              description='Cua gat: |do cong phia truoc| (1/m) tu muc nay VA banh da be >= 8 do cung chieu '
                                          '(0.15 s) moi bam vach ngoai, giu toi khi banh ve < 6 do; cua nhe / truoc '
                                          'khi vao cua giu giua lan'),
        DeclareLaunchArgument('corner_outer_gap_m', default_value='0.08',
                              description='CUA GAT: mep xe cach MEP bang keo vach NGOAI bay nhieu m, do truc tiep tu '
                                          'vach ngoai (khong theo be rong lan). Cham vach ngoai -> tang (0.10); van de '
                                          'vach trong -> giam (0.05); < 0 = tat'),
        DeclareLaunchArgument('kink_radius_m', default_value='1.0',
                              description='Goc gap: chi bat dau be lai khi truc sau con cach dinh R*tan(goc/2) '
                                          '(45 do: 0.41 m); truoc do di thang theo doan truoc goc. Van vao cua '
                                          'som / de vach trong -> giam (0.8); vot ra vach ngoai -> tang (1.3); 0 = tat'),
        DeclareLaunchArgument('corner_outer_max_m', default_value='0.15',
                              description='Doi duong bam ra phia vach ngoai toi da bay nhieu m'),
        DeclareLaunchArgument('odom_speed_scale', default_value='0.30',
                              description='Toc do THAT / toc do lenh, chi cho odometry (bo nho duong). Do tu anh 10/10: '
                                          'lenh 5.8 km/h -> that ~0.44 m/s (x0.27). 1.0 = nhu cu (bo nho troi nhanh 3.5 lan, '
                                          'be lai som)'),
        DeclareLaunchArgument('servo_straight_deg', default_value='90.0',
                              description='Goc servo (do) ma banh xe di THANG. Log 10/10 18:05 chay 2 chieu: servo 90 xe '
                                          'da re phai ~7.5 do servo -> cua phai be som de vach trong, cua trai thieu lai. '
                                          '90 = khong bu (mac dinh: ke xe len servo 90 banh thang). Chi de thu'),
        DeclareLaunchArgument('lost_memory_ms', default_value='1500',
                              description='Mat ca 2 vach: chay tiep theo duong da nho toi da (ms)'),
        DeclareLaunchArgument('steer_filter_s', default_value='0.08',
                              description='Loc goc banh (s), = STEER_FILTER_SEC ban Python; 0 = tat'),
        DeclareLaunchArgument('steer_rate_dps', default_value='15',
                              description='Toc do danh lai toi da tren duong THANG (do banh/s). Log 10/10 19:27 '
                                          'khong gioi han: 150-270 do/s, xe chao. Van chao -> giam; 0 = tat'),
        DeclareLaunchArgument('steer_rate_corner_dps', default_value='45',
                              description='Toc do danh lai toi da TRONG CUA (do banh/s): 45 = 0 -> het lai (21 do) '
                                          'trong ~0.5 s. Vao cua tre / be khong kip -> tang'),
        DeclareLaunchArgument('lane_start_frames', default_value='3',
                              description='Bam chay / mat lan dung xe: chay (lai) khi thay lan (2 vach '
                                          'hoac 1 vach on dinh) bay nhieu frame lien tiep (= LANE_START_FRAMES)'),
        DeclareLaunchArgument('single_search_m', default_value='0.08',
                              description='Chi thay 1 vach: doi tam bam ve phia vach bi mat toi da (m) -> '
                                          'xe lai vao trong tim lai vach kia; 0 = tat'),
        DeclareLaunchArgument('record_frames', default_value='true',
                              description='Ghi anh camera + trang thai lan moi frame khi xe CHAY (de chinh xu ly anh)'),
        DeclareLaunchArgument('record_dir', default_value='auto',
                              description='Thu muc goc ghi anh, auto = ~/lane_logs (moi lan chay 1 thu muc run_*)'),
        DeclareLaunchArgument('single_search_rate', default_value='0.30',
                              description='Toc do doi tam khi tim vach (m/s)'),
        DeclareLaunchArgument('xte_gain', default_value='2.0',
                              description='Phan hoi lech ngang tai chan camera (1/s); 0 = pure pursuit thuan'),
        DeclareLaunchArgument('xte_ki', default_value='50.0',
                              description='Tich phan lech ngang (do banh / m / s): bu trim servo, camera lap lech'),
        DeclareLaunchArgument('xte_i_max_deg', default_value='6.0',
                              description='Gioi han khau tich phan (do banh) (= STEER_I_LIMIT Python)'),
        DeclareLaunchArgument('xte_far_m', default_value='0.05',
                              description='Lech tam xa hon muc nay (m) thi keo ve manh hon (xem xte_far_gain)'),
        DeclareLaunchArgument('pid_kp', default_value='54',
                              description='PID DUONG THANG: do banh / m lech tam (54 = 0.54 do/cm). '
                                          'Ve tam cham -> tang; lac -> giam; 0 = tat PID (pure pursuit nhu cu)'),
        DeclareLaunchArgument('pid_kd', default_value='1.0',
                              description='PID: do banh / do lech HUONG xe so voi lan (khau D, giam chan). '
                                          'Van vot qua tam -> tang; banh run theo nhieu -> giam'),
        DeclareLaunchArgument('pid_ki', default_value='10',
                              description='PID: tich phan lech (do banh / m / s), chi tich khi xe gan tam tren thang'),
        DeclareLaunchArgument('pid_i_max_deg', default_value='3'),
        DeclareLaunchArgument('pid_max_deg', default_value='15',
                              description='PID: goc banh toi da tren duong thang (do)'),
        DeclareLaunchArgument('pid_filter_s', default_value='0.12',
                              description='PID: loc lech / huong (s); banh run -> tang, phan ung tre -> giam'),
        DeclareLaunchArgument('xte_far_gain', default_value='0.0',
                              description='Phan lech vuot xte_far_m duoc nhan them he so nay (1/s, cong vao xte_gain): '
                                          'lech 15 cm ve tam nhanh hon, lech nho giu nhu cu. 0 = tat (10/10 19:27: 2.0 xe chao tren thang)'),
    ]

    return LaunchDescription(declare + [OpaqueFunction(function=setup)])

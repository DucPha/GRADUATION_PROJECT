"""Wrapper launch cho Mini PC.

Toàn bộ cấu hình node nằm trong `bringup.launch.py`. File này chỉ include lại
nó để giữ tên lệnh cũ (`ros2 launch autonomous_vehicle minipc.launch.py`)
vẫn chạy được. Trước đây hai file khai báo node riêng nên tham số lệch nhau
(khác nhau ngay ở `camera_fps`: 30 vs 120, và `viz_hz`: 30 vs 120).

Lưu ý: KHÔNG truyền `launch_arguments` rỗng - nếu làm vậy sẽ override tất cả
các default value thành chuỗi rỗng, khiến node bị crash do ParameterTypeException.
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "bringup.launch.py")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            # Không truyền launch_arguments => giữ nguyên tất cả default values
            # từ bringup.launch.py. Truyền rỗng ("".split()) sẽ GHI ĐÈ tất cả
            # thành chuỗi rỗng và làm node C++ crash ngay lúc khởi động.
        )
    ])
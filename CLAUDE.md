# CLAUDE.md

## Project Context
Autonomous vehicle graduation project. **Mini PC (x86/ARM64, Linux/ROS2) + ESP32-S3**.
- Languages: C++ (ROS2 nodes, Arduino), Python (NCNN inference)
- Hardware: camera lane detection, RPLiDAR obstacle avoidance, DC motors, servos
- Comms: UART serial between Mini PC ↔ ESP32-S3, Wi-Fi OTA
- Build: colcon build (ROS2), Arduino IDE / PlatformIO (ESP32)

## Core Rules
> Follow ./skills/karpathy-guidelines/ for: Think Before Coding, Simplicity First, Surgical Changes, Goal-Driven Execution.
> Follow ./skills/superpowers/ debugging protocol before any bug fix or feature.
> Follow ./skills/planning-with-files/ for non-trivial bugs/features and task tracking.

## Output Discipline
- Vietnamese. Concise. No pleasantries.
- Output only the terminal command or code diff needed.
- Explain cause in ≤2 sentences.
- Keep replies short and practical.

## Embedded-Specific Rules
- Memory: no dynamic allocation in ISR. Prefer stack/static. Check RAM on ESP32 (320KB).
- Timing: mark blocking calls. Use millis() instead of delay() in loops. Respect RTOS task priorities.
- Serial protocol: preserve the exact packet layouts. Document any change.
  MiniPC → ESP32 (11 bytes): `AB CD | dev_hi dev_lo | speed(km/h×10) | emg | 00 00 00 00 | XOR(byte 2..9)`
  ESP32 → MiniPC (7 bytes): `DC BA | float velocity (4 bytes) | XOR(byte 2..5)`
  Both sides hard-code 230400 baud. Steering sign: `dev < 0` = left, `dev > 0` = right.
- Pin assignments: never change GPIO mapping without explicit request.
- Safety: obstacle avoidance logic is safety-critical. Test sensor timeout, NaN, motor stall, and stale data cases.

## File Layout
/
  README.md                    ← project map and Linux/ROS2 setup
  CLAUDE.md                    ← rules for AI assistants working on this repo
  Dockerfile                   ← container image (ROS_DISTRO build arg, default jazzy)
  docker-entrypoint.sh         ← container entrypoint
  rosdep.yaml                  ← rosdep keys for `rosdep install --from-paths src`
  src/
    autonomous_vehicle/
      include/ src/            ← main C++ ROS 2 package: camera + LiDAR + obstacle avoidance + UART
      launch/bringup.launch.py ← the ONLY launch file that declares nodes
      launch/minipc.launch.py  ← thin wrapper around bringup.launch.py
      rviz/autonomous.rviz     ← rviz config, resolved via get_package_share_directory
      test/                    ← ament_add_gtest unit tests
    traffic_light_detector/    ← NCNN traffic-light detector
    turn_detector/             ← NCNN turn-direction detector
  firmware/
    esp32s3/                    ← active ESP32-S3 sketch
    legacy/                     ← previous firmware variants and hardware tests
  docs/                         ← project documentation (incl. FIX_REPORT.md)
  tools/                        ← camera and development utilities
  skills/                       ← coding and debugging skill definitions
  build/install/log/            ← generated output; do not edit

## Build & Test
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select autonomous_vehicle --cmake-args -DBUILD_TESTING=ON
./build/autonomous_vehicle/test_lidar_module   # chạy trực tiếp, nhanh hơn colcon test
```

## Known Hardware Caveats (verify on the real car before trusting)
- `PIN_STEER = 33` on ESP32-S3 conflicts with octal PSRAM when PSRAM is enabled.
- Wheel math constants must be measured: `WHEEL_DIA_M`, `PULSES_PER_REV`, `GEAR_RATIO`.

## Practical Rule
Use only the project-relevant coding skills above. Remove unrelated skills and generated artifacts to save tokens and keep the workspace clean.

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
- Serial protocol: preserve existing packet format [START][CMD][LEN][DATA][CRC][END]. Document any changes.
- Pin assignments: never change GPIO mapping without explicit request.
- Safety: obstacle avoidance logic is safety-critical. Test sensor timeout, NaN, motor stall, and stale data cases.

## File Layout
/
  README.md                    ← project map and Linux/ROS2 setup
  src/
    PhamMinhDuc/                ← main C++ ROS 2 package: camera + LiDAR + obstacle avoidance + UART
    traffic_light_detector/    ← NCNN traffic-light detector
    turn_detector/             ← NCNN turn-direction detector
  firmware/
    esp32s3/                    ← active ESP32-S3 sketch
    legacy/                     ← previous firmware variants and hardware tests
  docs/                         ← project documentation
  tools/                        ← camera and development utilities
  skills/                       ← coding and debugging skill definitions
  build/install/log/            ← generated output; do not edit

## Practical Rule
Use only the project-relevant coding skills above. Remove unrelated skills and generated artifacts to save tokens and keep the workspace clean.

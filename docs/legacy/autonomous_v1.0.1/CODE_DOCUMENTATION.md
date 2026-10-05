# Tài liệu Code - autonomous_v1.0.1.ino

## Mục lục
- [Hằng số cấu hình](#hằng-số-cấu-hình)
- [Biến toàn cục](#biến-toàn-cục)
- [Hàm xử lý](#hàm-xử-lý)

---

## Hằng số cấu hình

### GPIO Pins
| Hằng số | Giá trị | Chức năng |
|---------|---------|-----------|
| `SERVO_PIN` | 32 | PWM điều khiển servo lái |
| `HALL_SENSOR_PIN` | 23 | Input đọc Hall sensor (đo tốc độ) |
| `SIGNAL_LEFT_PIN` | 21 | Output đèn xi-nhan trái |
| `SIGNAL_RIGHT_PIN` | 19 | Output đèn xi-nhan phải |
| `BRAKE_PIN` | 15 | Output đèn phanh |
| `ESC_PIN` | 22 | PWM điều khiển ESC motor |

### Servo & Steering
| Hằng số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| `SERVO_CENTER` | 90 | Góc servo đi thẳng (độ) |
| `SERVO_MIN` | 45 | Giới hạn góc rẽ phải tối đa |
| `SERVO_MAX` | 135 | Giới hạn góc rẽ trái tối đa |
| `MAX_STEER_ANGLE` | 45 | Biên độ rẽ tối đa từ tâm (±45°) |
| `DEADZONE` | 10 | Vùng chết lane deviation (pixel) |
| `DEVIATION_MAX` | 50 | Độ lệch làn tối đa (pixel) |
| `TURN_THRESHOLD` | 10 | Ngưỡng bật xi-nhan (pixel) |

### PID Gains - Steering
| Hằng số | Giá trị | Dùng khi |
|---------|---------|----------|
| `KP_LOW`, `KI_LOW`, `KD_LOW` | 1.5, 0.08, 0.6 | Tốc độ < 10 km/h |
| `KP_MED`, `KI_MED`, `KD_MED` | 1.0, 0.05, 0.9 | Tốc độ 10-25 km/h |
| `KP_HIGH`, `KI_HIGH`, `KD_HIGH` | 0.6, 0.02, 1.3 | Tốc độ > 25 km/h |

### Filter & Timing
| Hằng số | Giá trị | Mục đích |
|---------|---------|----------|
| `FILTER_SIZE` | 5 | Kích thước buffer moving average |
| `USE_MEDIAN_FILTER` | true | Bật median filter 3 mẫu |
| `SPEED_CALC_INTERVAL_MS` | 100 | Chu kỳ tính tốc độ (ms) |
| `SPEED_TIMEOUT_MS` | 500 | Timeout Hall sensor → speed=0 |
| `SIGNAL_BLINK_TIME` | 500 | Chu kỳ nhấp nháy xi-nhan (ms) |

### Serial Protocol
| Hằng số | Giá trị | Giải thích |
|---------|---------|------------|
| `PREAMBLE` | 0xABCD | Đồng bộ đầu packet |
| `BAUDRATE` | 230400 | Tốc độ UART |
| `SERIAL_TIMEOUT_MS` | 5 | Timeout đọc serial |
| `COMM_TIMEOUT_MS` | 2000 | Mất liên lạc → emergency stop |

### ESC Motor Control
| Hằng số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| `ESC_STOP` | 90 | Góc ESC dừng hẳn |
| `ESC_DEADZONE_END` | 94 | Cuối vùng chết ESC |
| `ESC_MIN_FWD` | 95 | Góc ESC tiến tối thiểu |
| `ESC_MAX_FWD` | 180 | Góc ESC tiến tối đa |
| `ESC_REVERSE_BRAKE` | 20 | Góc ESC lùi (dùng phanh) |

### Speed Calculation
| Hằng số | Giá trị | Giải thích |
|---------|---------|------------|
| `MAX_SPEED_KMH` | 15.0 | Giới hạn tốc độ tối đa |
| `GEAR_RATIO_MOTOR_TO_WHEEL` | 1.0 | Tỷ số truyền motor → differential |
| `DIFF_GEAR_LARGE` | 37.0 | Số răng bánh lớn differential |
| `DIFF_GEAR_SMALL` | 13.0 | Số răng bánh nhỏ differential |
| `DIFF_RATIO` | 2.846 | = 37/13 |
| `TOTAL_GEAR_RATIO` | 2.846 | Tỷ số truyền tổng |
| `WHEEL_DIAMETER_M` | 0.090 | Đường kính bánh xe (m) |
| `WHEEL_CIRCUMFERENCE_M` | 0.283 | Chu vi bánh xe (π × 0.09) |
| `HALL_PULSES_PER_DIFF_REV` | 4 | Số xung Hall/vòng trục differential |

---

## Biến toàn cục

### Objects
```cpp
Servo steeringServo;  // Đối tượng servo lái
Servo escMotor;       // Đối tượng ESC motor
```

### Steering PID State
```cpp
float Kp, Ki, Kd;              // Gains PID hiện tại (thay đổi theo tốc độ)
float integral = 0.0;          // Tích lũy sai số steering
float previous_error = 0.0;    // Sai số lần trước (tính đạo hàm)
unsigned long last_pid_time;   // Timestamp tính PID lần trước
```

### Vehicle State
```cpp
int16_t current_deviation = 0;     // Độ lệch làn đường từ camera (pixel)
float target_speed_kmh = 0.0;      // Tốc độ mục tiêu từ miniPC
float current_speed_kmh = 0.0;     // Tốc độ thực tế đo được
float current_steering_angle = 90.0; // Góc servo hiện tại (độ)
int current_esc_angle = 90;        // Góc ESC hiện tại (độ)
```

### Speed PID State
```cpp
float speed_integral = 0.0;        // Tích lũy sai số speed control
float prev_speed_error = 0.0;      // Sai số tốc độ lần trước
```

### Hall Sensor & Speed Calculation (ISR-shared)
```cpp
volatile unsigned long pulse_count = 0;        // Đếm xung Hall (ISR write, main read)
volatile unsigned long last_pulse_time_us = 0; // Timestamp xung cuối (μs)
unsigned long last_speed_calc = 0;             // Lần tính tốc độ cuối
```

### Speed Filtering
```cpp
float rpm_buffer[FILTER_SIZE];     // Buffer moving average (5 mẫu)
int rpm_buffer_index = 0;          // Chỉ số vòng tròn buffer
bool rpm_buffer_filled = false;    // Buffer đã đầy chưa
```

### Signal Lights
```cpp
unsigned long last_blink_time = 0;  // Timestamp nhấp nháy cuối
bool blink_state = false;           // Trạng thái LED (ON/OFF)
bool left_signal_active = false;    // Xi-nhan trái có đang bật
bool right_signal_active = false;   // Xi-nhan phải có đang bật
```

### Communication & Safety
```cpp
unsigned long packet_count = 0;      // Tổng số packet hợp lệ nhận được
unsigned long error_count = 0;       // Tổng số packet lỗi
unsigned long last_packet_time = 0;  // Timestamp packet cuối
bool emergency_stop_flag = false;    // Cờ phanh khẩn cấp
bool brake_in_progress = false;      // Đang trong quá trình phanh
```

### Lookup Table
```cpp
const ESCSpeedPoint esc_lut_loaded[21]; // Bảng ESC → tốc độ (đo thực nghiệm)
const int esc_lut_size = 21;            // Số điểm trong LUT
```

---

## Hàm xử lý

### `void IRAM_ATTR hallSensorISR()`
**Loại:** Interrupt Service Routine (ISR)  
**Trigger:** Cạnh lên (RISING) pin Hall sensor

**Chức năng:**
- Đếm xung từ Hall sensor để tính tốc độ
- Debounce: bỏ qua xung cách nhau < 1ms (chống nhiễu)
- Cập nhật `pulse_count` và `last_pulse_time_us`

**Lưu ý:** 
- Biến `volatile` để đồng bộ với main loop
- Code trong ISR phải cực kỳ ngắn gọn

---

### `float medianFilter(float new_value)`
**Input:** Giá trị RPM mới  
**Output:** Giá trị trung vị của 3 mẫu gần nhất

**Chức năng:**
- Duy trì buffer 3 mẫu `[n-2, n-1, n]`
- Sắp xếp bubble sort và lấy phần tử giữa
- Loại bỏ nhiễu spike từ điện từ hoặc lỗi đọc

**Ví dụ:**
```
Input sequence: 100, 250 (spike), 105
Output: median([100, 250, 105]) = 105  ← spike bị loại
```

---

### `float movingAverageFilter(float new_value)`
**Input:** Giá trị RPM đã qua median filter  
**Output:** Trung bình động của 5 mẫu gần nhất

**Chức năng:**
- Buffer vòng tròn kích thước 5
- Trả về trung bình các mẫu hợp lệ (< 5 lúc khởi động)
- Làm mượt dao động tốc độ

---

### `void calculateSpeed()`
**Gọi:** Mỗi loop (~mỗi 20ms)  
**Tính toán:** Mỗi 100ms

**Công thức:**
```
1. differential_rpm = (pulse_count / 4) × (60 / Δt)
2. filtered_rpm = movingAverage(medianFilter(differential_rpm))
3. wheel_rpm = filtered_rpm / TOTAL_GEAR_RATIO
4. speed_kmh = (wheel_rpm × WHEEL_CIRCUMFERENCE × 60) / 1000
```

**Timeout logic:**
- Nếu không có xung trong 500ms → tốc độ = 0, xóa buffer

---

### `float lookupESCFromSpeed(float target_speed)`
**Input:** Tốc độ mục tiêu (km/h)  
**Output:** Góc ESC tương ứng (độ)

**Chức năng:**
- Tìm kiếm trong bảng LUT 21 điểm
- Nội suy tuyến tính giữa 2 điểm
- Xử lý đặc biệt:
  - `< 0.5 km/h` → ESC_STOP (90°)
  - `< 1.55 km/h` → ESC_MIN_FWD (95°)

**Ví dụ:**
```
Target: 7.0 km/h
LUT[i]:   ESC=102°, speed=6.56 km/h
LUT[i+1]: ESC=105°, speed=8.34 km/h

ratio = (7.0 - 6.56) / (8.34 - 6.56) = 0.247
ESC = 102 + 0.247 × (105 - 102) = 102.74°
```

---

### `void performEmergencyBrake()`
**Gọi:** Khi `emergency_stop_flag = true` và xe đang chạy

**Logic pulse braking:**
```
Lặp 10 chu kỳ:
  Phase 1: ESC = 20° (reverse) trong 40ms
  Phase 2: ESC = 94° (neutral) trong 10ms
Sau 10 chu kỳ: ESC = 90° (stop), reset PID
```

**State variables:**
- `brake_in_progress`: đang phanh hay không
- `brake_start_time`: timestamp bắt đầu phase hiện tại
- `brake_pulse_count`: số chu kỳ đã hoàn thành
- `pulse_phase`: đang ở phase reverse (true) hay neutral (false)

**Lý do:** ESC không chịu được reverse liên tục → dễ cháy mạch

---

### `int calculateSpeedPID(float target_speed_kmh, float current_speed_kmh)`
**Input:** Tốc độ mục tiêu, tốc độ hiện tại  
**Output:** Góc ESC sau khi điều chỉnh PID

**Flow:**
1. Nếu `target < 0.5` → reset PID, trả về 90°
2. Lookup target ESC từ LUT
3. Chọn gains (Kp, Ki, Kd) dựa trên **tốc độ hiện tại**:
   - `≤ 3.58 km/h`: Kp cao (1.2-1.5), khởi động mạnh
   - `3.58-8 km/h`: Kp thấp (0.5), tránh overshoot
   - `8-12 km/h`: Kp trung bình (1.0)
   - `> 12 km/h`: Kp cao (1.2), duy trì tốc độ
4. Tính PID:
   ```
   error = target_speed - current_speed
   P = Kp × error
   I = Ki × integral
   D = Kd × (error - prev_error) / dt
   ```
5. Anti-windup:
   ```
   if (current > target):
     integral *= 0.2  // reset nhanh khi overshoot
   else:
     integral += error × 0.02
   ```
6. Clamp ESC change: giới hạn tốc độ thay đổi (`max_change`)
7. Clamp output: `current_esc ∈ [90, target_esc + margin]`

**Đặc biệt vùng thấp (≤ 3.58 km/h):**
- Nếu `error < -0.3` (chạy quá nhanh) → Kp = 1.5 (phanh mạnh)
- Ngược lại → Kp = 1.2 (tăng tốc nhẹ nhàng)

---

### `void selectPIDGains(float speed)`
**Input:** Tốc độ hiện tại  
**Output:** Cập nhật `Kp`, `Ki`, `Kd` toàn cục (cho steering)

**Logic:**
```cpp
if (speed < 10.0)   → LOW gains  (nhạy)
else if (< 25.0)    → MED gains  (cân bằng)
else                → HIGH gains (ổn định)
```

---

### `float calculatePID(float error)`
**Input:** Sai số góc lái (target_angle - 90°)  
**Output:** Độ hiệu chỉnh góc servo

**Công thức chuẩn:**
```cpp
P = Kp × error
I = Ki × integral (clamped ±15)
D = Kd × (error - previous_error) / dt
return P + I + D
```

**Tính dt:** 
- `dt = (millis() - last_pid_time) / 1000.0`
- Nếu `dt ≤ 0` hoặc `> 1.0` → mặc định 0.02s

---

### `float deviationToBaseAngle(int16_t dev)`
**Input:** Độ lệch làn (pixel, âm=trái, dương=phải)  
**Output:** Góc servo cơ sở trước khi PID

**Mapping:**
```
dev ∈ [-10, +10] → 90° (deadzone, đi thẳng)
dev < -10        → 90° + map(|dev|, 10→50, 0→45°)  // rẽ phải
dev > +10        → 90° - map(dev, 10→50, 0→45°)    // rẽ trái
```

**Ví dụ:**
```
dev = -30 → norm = 30-10 = 20
          → angle = 90 + map(20, 0→40, 0→45) = 90 + 22.5 = 112.5°
```

---

### `void controlSteering(int16_t dev)`
**Input:** Độ lệch làn từ camera  
**Chức năng:** Điều khiển servo lái với PID cascade

**Flow:**
1. Chọn gains dựa trên tốc độ (`selectPIDGains`)
2. Chuyển deviation → base angle (`deviationToBaseAngle`)
3. Tính error = base - 90°
4. PID correction (`calculatePID`)
5. Final angle = 90° + correction
6. Clamp vào `[45°, 135°]` và ghi servo
7. Lưu `current_steering_angle`

---

### `void updateSignalLights()`
**Gọi:** Mỗi loop

**Logic:**
- Nhấp nháy mỗi 500ms (`blink_state` toggle)
- **Nếu `|deviation| < 10`**: tắt cả 2 đèn
- **Nếu `deviation > 10`**: đèn trái nhấp nháy
- **Nếu `deviation < -10`**: đèn phải nhấp nháy

---

### `void updateBrakeLight()`
**Gọi:** Mỗi loop

**Sáng đèn khi:**
- `target_speed < 0.5` (dừng)
- `emergency_stop_flag = true`
- `brake_in_progress = true`

---

### `void sendFeedback()`
**Gọi:** Mỗi 33ms (~30Hz)

**Packet gửi (7 bytes):**
```
[0xDC][0xBA][speed_float_32bit][checksum]
```
- Checksum: XOR 4 bytes của float
- Gửi khi `Serial.availableForWrite() >= 7`

---

### `void receiveCommands()`
**Gọi:** Mỗi loop

**Flow:**
1. **Buffer overflow protection**: nếu `available() > 100` → flush
2. **Đọc 11 bytes** nếu đủ:
   ```
   [0xAB][0xCD][dev_hi][dev_lo][speed][emergency][pad×4][checksum]
   ```
3. **Kiểm tra preamble**: `(buf[0] << 8) | buf[1] == 0xABCD`
4. **Parse data:**
   ```cpp
   deviation = (int16_t)((buf[2] << 8) | buf[3])
   target_speed = buf[4] / 10.0
   emergency_flag = (buf[5] != 0)
   ```
5. **Checksum**: XOR buf[2→9] == buf[10]
6. **Clamp speed**: `min(target_speed, MAX_SPEED_KMH)`
7. **Timeout check**: 
   ```cpp
   if (no_packet_for_2000ms):
     target_speed = 0
     deviation = 0
     emergency_flag = true
   ```

---

### `void printDebugInfo()`
**Gọi:** Mỗi 200ms

**In ra Serial:**
```
Target: X.XX km/h, Speed: Y.YY km/h, ESC: ZZZ°, 
ESC_Kp: a.aaa, ESC_Ki: b.bbb, ESC_Kd: c.cccc,
Steer: DD.D°, Steer_Kp: e.ee, Steer_Ki: f.fff, Steer_Kd: g.gg
```

**Lưu ý:** Hàm này dùng `Serial.print` → blocking → có thể gây jitter nếu packet dài

---

### `void setup()`
**Chạy:** 1 lần lúc khởi động

**Khởi tạo:**
1. Serial 230400 baud, timeout 5ms
2. Allocate 4 timers PWM cho ESP32
3. Setup GPIO:
   - `HALL_SENSOR_PIN`: INPUT_PULLUP
   - Các LED: OUTPUT
4. Attach servo:
   - Steering: 50Hz, pulse 500-2400μs
   - ESC: pulse 1000-2000μs
5. Đặt vị trí trung tính:
   - Servo = 90°
   - ESC = 90° (stop)
6. Attach interrupt Hall sensor: RISING edge → `hallSensorISR()`
7. Tắt tất cả LED
8. Xóa buffer RPM
9. Init timestamps
10. In banner debug

---

### `void loop()`
**Chu kỳ:** ~20ms (không có delay, chạy tối đa tốc độ)

**Flow chính:**
```cpp
1. calculateSpeed()           // Tính tốc độ từ Hall sensor
2. receiveCommands()          // Đọc lệnh từ miniPC

3. Xử lý emergency:
   if (emergency_stop_flag):
     if (cần bắt đầu phanh):
       performEmergencyBrake()  // Khởi động pulse braking
     if (đang phanh):
       performEmergencyBrake()  // Tiếp tục pulse
     else:
       ESC = 90° (stop)
   else:
     ESC = calculateSpeedPID() // Điều khiển tốc độ bình thường
     
4. controlSteering(deviation) // Điều khiển lái
5. updateSignalLights()       // Cập nhật xi-nhan
6. updateBrakeLight()         // Cập nhật đèn phanh

7. Mỗi 33ms:
   sendFeedback()             // Gửi tốc độ về miniPC
   
8. Mỗi 200ms:
   printDebugInfo()           // In debug log
```

---

## Struct & Type

### `struct ESCSpeedPoint`
```cpp
struct ESCSpeedPoint {
  int esc;      // Góc ESC (độ)
  float speed;  // Tốc độ tương ứng (km/h)
};
```

**Dùng cho:** Lookup table ánh xạ ESC → tốc độ thực tế (đo từ thực nghiệm).

**Ví dụ:**
```cpp
{90, 0.00},   // Dừng
{95, 1.55},   // Khởi động
{180, 14.75}  // Tối đa
```

---

## Sơ đồ luồng dữ liệu

```
┌─────────────────────────────────────────────────────────┐
│                  UART từ miniPC                         │
│  [deviation, target_speed, emergency_flag] ─────┐       │
└──────────────────────────────────────────────────┼───────┘
                                                   │
                                                   ▼
┌──────────────────────────────────────────────────────────┐
│                   receiveCommands()                      │
│  - Parse packet 11 bytes                                 │
│  - Checksum validation                                   │
│  - Timeout detection (2s)                                │
└──────────────┬───────────────────────────────────────────┘
               │
               ├──────────────────────┬──────────────────────┐
               ▼                      ▼                      ▼
       current_deviation      target_speed_kmh      emergency_stop_flag
               │                      │                      │
               ▼                      ▼                      ▼
    ┌──────────────────┐   ┌──────────────────┐   ┌─────────────────┐
    │ controlSteering()│   │calculateSpeedPID()│   │performEmergency │
    │  - deviation→    │   │  - PID 4 vùng    │   │     Brake()     │
    │    base angle    │   │  - LUT lookup    │   │  - Pulse 10×    │
    │  - PID cascade   │   │  - Anti-windup   │   │  - 40ms reverse │
    │  - Gain schedule │   │  - ESC clamp     │   │  - 10ms neutral │
    └────────┬─────────┘   └────────┬─────────┘   └────────┬────────┘
             │                      │                       │
             ▼                      ▼                       ▼
      steeringServo.write()   escMotor.write()      escMotor.write()
             │                      │                       │
             └──────────────────────┴───────────────────────┘
                                    │
                                    ▼
                            ┌───────────────┐
                            │  Actuators:   │
                            │  - Servo lái  │
                            │  - ESC motor  │
                            └───────────────┘

┌──────────────────────────────────────────────────────────┐
│               Hall Sensor (Interrupt)                    │
│  hallSensorISR() → pulse_count++                         │
└──────────────┬───────────────────────────────────────────┘
               │
               ▼
       calculateSpeed()
         - Median filter (3 mẫu)
         - Moving average (5 mẫu)
         - Gear ratio conversion
               │
               ▼
       current_speed_kmh ────────────┐
                                     │
                                     ▼
                             ┌───────────────┐
                             │ sendFeedback()│
                             │   (30Hz)      │
                             └───────┬───────┘
                                     │
                                     ▼
                            UART → miniPC
```

---

## Timing Diagram

```
Time →  0ms    33ms   66ms   100ms  200ms  2000ms
        │      │      │      │      │      │
loop()  ├──────┼──────┼──────┼──────┼──────┤  (continuous, ~20ms/loop)
        │      │      │      │      │      │
sendFb  ●------●------●------●------●      │  (every 33ms)
        │      │      │      │      │      │
calcSpd │      │      │      ●------│      │  (every 100ms)
        │      │      │      │      │      │
debug   │      │      │      │      ●------│  (every 200ms)
        │      │      │      │      │      │
timeout │      │      │      │      │      ●  (no packet → emergency)
```

---

## Các lưu ý quan trọng

### ⚠️ Race Conditions
- `pulse_count` được ghi từ ISR, đọc từ main → cần `noInterrupts()`/`interrupts()`
- Biến `volatile` để tránh compiler optimization

### ⚠️ Timing-sensitive
- `calculateSpeed()` dựa vào `pulse_count` mỗi 100ms
- Nếu loop bị block (VD: Serial.print dài) → đọc sai tốc độ

### ⚠️ ESC Deadzone
- ESC có vùng chết 90-94° (không chuyển động)
- Code đảm bảo không output góc trong vùng này

### ⚠️ Buffer Overflow
- `receiveCommands()` flush khi buffer > 100 bytes
- Tránh bộ nhớ UART tràn làm mất đồng bộ packet

### ⚠️ Emergency Recovery
**VẤN ĐỀ**: Khi `emergency_stop_flag = true` do timeout, flag không tự reset khi miniPC reconnect
→ Xe dừng mãi mãi ngay cả khi nhận packet mới

**GIẢI PHÁP:** Cần thêm `emergency_stop_flag = false` sau khi parse packet hợp lệ (line 476).

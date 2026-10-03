// ============================================================================
// BỘ ĐIỀU KHIỂN XE TỰ HÀNH ESP32-S3 (AUTONOMOUS VEHICLE)
// Phân hệ: Low-level Controller (Xử lý tín hiệu & Chấp hành)
// Kiến trúc: Real-time, CPU Budgeting, PID 100Hz, Spinlock MUX
// Tối ưu hóa: Adaptive EMA Filter, Toán học Compile-time, Non-blocking UART
// ============================================================================

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================================
// [1] CẤU HÌNH CHÂN PHẦN CỨNG (PINS)
// ============================================================================
constexpr uint8_t PIN_STEER = 32;  // Chân PWM xuất ra Servo bẻ lái
constexpr uint8_t PIN_HALL = 23;   // Chân ngắt đọc xung Cảm biến Hall đo tốc độ
constexpr uint8_t PIN_TURN_L = 21; // Đèn LED xi-nhan trái
constexpr uint8_t PIN_TURN_R = 18; // Đèn LED xi-nhan phải (Tránh chân 19 là USB_D- của ESP32-S3)
constexpr uint8_t PIN_BRAKE = 15;  // Đèn LED phanh đít
constexpr uint8_t PIN_ESC = 33;    // Chân PWM xuất ra Động cơ (ESC)

// ============================================================================
// [2] THÔNG SỐ VẬT LÝ & ĐIỀU KHIỂN (PARAMETERS)
// ============================================================================
// --- Hệ thống lái (Steering) ---
constexpr int STEER_CENTER = 90; // Góc Servo khi xe đi thẳng
constexpr int STEER_MIN = 45;    // Giới hạn góc Servo rẽ phải tối đa
constexpr int STEER_MAX = 135;   // Giới hạn góc Servo rẽ trái tối đa

constexpr float CAM_DEADZONE = 10.0f; // Vùng chết (pixel): Lệch dưới mức này xem như đi thẳng
constexpr float CAM_MAX_DEV = 50.0f;  // Độ lệch quỹ đạo tối đa (pixel) để quy đổi ra góc lái
constexpr float BLINK_THRESH = 10.0f; // Ngưỡng lệch làn (pixel) để tự động bật xi-nhan

// --- Động cơ (Speed/ESC) ---
constexpr int ESC_NEUTRAL = 90;  // Xung PWM để ESC dừng động cơ (Số Mo)
constexpr int ESC_MIN_FWD = 95;  // Xung PWM tối thiểu để xe bắt đầu nhích tới
constexpr int ESC_MAX_FWD = 180; // Xung PWM tối đa cho chiều tiến
constexpr int ESC_BRAKE = 20;    // Xung PWM kéo lùi để hãm phanh điện từ

constexpr float MAX_SPEED_KMH = 15.0f;       // Giới hạn tốc độ mục tiêu an toàn từ MiniPC
constexpr float MAX_VALID_SPEED_KMH = 20.0f; // Ngưỡng chặn nhiễu sinh tốc độ ảo vượt 20km/h

// --- Toán học tính vận tốc (Compile-time) ---
constexpr float WHEEL_DIA_M = 0.090f;             // Đường kính bánh xe (90mm = 0.09m)
constexpr float WHEEL_CIRCUMF = PI * WHEEL_DIA_M; // Chu vi bánh xe (m)
constexpr float GEAR_RATIO = 37.0f / 13.0f;       // Tỷ số truyền vi sai
constexpr uint8_t PULSES_PER_REV = 4;             // Số lượng xung Hall / 1 vòng quay bánh

// Hằng số quy đổi từ Chu kỳ (us) sang Tốc độ (km/h) = (3.6 * 1,000,000 * Chu vi) / (Xung * Tỷ số)
constexpr float PERIOD_TO_KMH_FACTOR = (3600000.0f * WHEEL_CIRCUMF) / (PULSES_PER_REV * GEAR_RATIO);
// Chu kỳ ứng với MAX_VALID_SPEED_KMH. CHỈ dùng để tài liệu hoá / kiểm tra,
// KHÔNG dùng làm điều kiện "có hợp lệ không" trong calcSpeed() - xem chú thích
// tại nơi dùng. Ngưỡng loại nhiễu thực sự là hằng dưới đây.
constexpr uint32_t MIN_HALL_PERIOD_US = (uint32_t)(PERIOD_TO_KMH_FACTOR / MAX_VALID_SPEED_KMH);
// Ngưỡng chặn nhiễu Hall: chu kỳ ngắn hơn 3 ms (>= 12000 km/h theo hằng số
// quy đổi) chắc chắn là nhiễu điện, không phải tốc độ thật.
constexpr uint32_t MIN_HALL_PERIOD_US_NOISE = 3000;

// --- Bộ lọc và Thời gian định thời (Timing & Filters) ---
constexpr uint32_t PID_DT_US = 10000;      // 100Hz: Chu kỳ định thời thuật toán PID
constexpr uint32_t TELEM_DT_MS = 20;       // 50Hz: Chu kỳ gửi phản hồi trạng thái về MiniPC
constexpr uint32_t WDOG_TIMEOUT_MS = 500;  // 0.5s: Nếu mất kết nối PC quá nửa giây -> Dừng xe
constexpr uint32_t BLINK_DT_MS = 500;      // 0.5s: Tốc độ nháy đèn xi-nhan

constexpr uint32_t HALL_DEBOUNCE_US = 1000; // 1ms: Lọc nhiễu cảm biến Hall
constexpr uint32_t BRAKE_HOLD_MS = 200;     // 200ms: Thời gian giữ lệnh lùi để hãm xe phanh
constexpr uint32_t FRAME_TIMEOUT_US = 10000;// 10ms: Timeout frame UART đứt đoạn

// Dưới ngưỡng này (km/h) coi như xe đã dừng hẳn => không cần phanh nữa.
constexpr float STOPPED_KMH = 0.3f;

constexpr float ALPHA_STEER = 0.4f; // Hệ số EMA làm mượt góc lái

// --- Giao tiếp (UART Protocol) ---
// LƯU Ý: UART này mang DUY NHẤT protocol nhị phân 7 byte (xem sendTelemetry).
// Tuyệt đối không Serial.print/printf ở đây: chữ ASCII chen vào giữa các gói
// làm MiniPC mất đồng bộ khung, và bản thân bộ đệm RX bị nhiễu. Cần xem trạng
// thái thì đọc qua topic telemetry phía MiniPC hoặc dùng cổng USB/Serial mặc
// định của board (USB CDC), không phải cổng này.
constexpr uint32_t UART_BAUD = 230400; // Khớp với BAUDRATE trong serial_esp32.hpp
constexpr size_t RX_BUF_SIZE = 4096;
constexpr size_t TX_BUF_SIZE = 512;

constexpr uint8_t HDR_RX1 = 0xAB, HDR_RX2 = 0xCD; // Byte đồng bộ nhận lệnh
constexpr uint8_t HDR_TX1 = 0xDC, HDR_TX2 = 0xBA; // Byte đồng bộ gửi Telemetry
constexpr uint8_t RX_LEN = 11;
constexpr uint8_t TX_LEN = 7;

constexpr uint32_t CPU_RX_BUDGET_US = 800; // 800us ngân sách xử lý UART mỗi loop
constexpr uint16_t MAX_RX_PER_LOOP = 2048;

constexpr uint8_t IDX_LANE_H = 2, IDX_LANE_L = 3;
constexpr uint8_t IDX_SPEED = 4, IDX_EMG = 5, IDX_CSUM = 10;

// ============================================================================
// [3] CẤU TRÚC DỮ LIỆU (DATA STRUCTURES)
// ============================================================================
struct CarState {
  float target_spd = 0.0f;
  float cur_spd = 0.0f;
  int16_t raw_dev = 0;
  float smooth_dev = 0.0f;
  int steer_cmd = STEER_CENTER;
  int esc_cmd = ESC_NEUTRAL;
  bool emg_stop = false;
  bool braking = false;
};

struct PidState {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float integral = 0.0f;
  float prev_err = 0.0f;
};

struct HallState {
  volatile uint32_t last_pulse = 0;
  volatile uint32_t period_us = 0;
  volatile uint32_t pulse_cnt = 0;
  volatile uint32_t sequence = 0;

  uint32_t proc_pulse_us = 0;
  uint32_t valid_period = 0;
  float smooth_speed_kmh = 0.0f;
  float saved_spd = 0.0f;
};

enum class RxState : uint8_t { WAIT_H1, WAIT_H2, READ_PAYLOAD };

// ============================================================================
// [4] BIẾN TOÀN CỤC (GLOBALS)
// ============================================================================
Servo servo_steer, motor_esc;

CarState car;
PidState pid_steer = {1.5f, 0.08f, 0.6f, 0.0f, 0.0f};
PidState pid_speed;
HallState hall;

portMUX_TYPE hall_mux = portMUX_INITIALIZER_UNLOCKED;

RxState rx_state = RxState::WAIT_H1;
uint8_t rx_buf[RX_LEN] = {};
uint8_t rx_idx = 0;
uint32_t last_rx_us = 0;
uint32_t last_packet_ms = 0;

uint32_t next_pid_us = 0;
bool pid_timer_init = false;

int last_esc = ESC_NEUTRAL;
int last_steer = STEER_CENTER;

struct EscMap {
  int pwm;
  float spd;
};
const EscMap ESC_LUT[] = {
    {90, 0.00f},   {94, 0.00f},   {95, 1.55f},   {100, 3.58f},  {102, 6.56f},
    {105, 8.34f},  {110, 9.50f},  {115, 11.0f},  {120, 12.37f}, {125, 12.79f},
    {130, 13.41f}, {135, 13.59f}, {140, 13.71f}, {145, 14.01f}, {150, 14.16f},
    {155, 14.19f}, {160, 14.28f}, {165, 14.31f}, {170, 14.34f}, {175, 14.46f},
    {180, 14.75f}};
constexpr size_t LUT_SIZE = sizeof(ESC_LUT) / sizeof(ESC_LUT[0]);

// ============================================================================
// [5] HÀM TIỆN ÍCH & NGẮT (UTILS & ISR)
// ============================================================================
inline float mapF(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  x = constrain(x, in_min, in_max);
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

inline uint8_t calcXor(const uint8_t *buf, uint8_t start, uint8_t end) {
  uint8_t csum = 0;
  for (uint8_t i = start; i <= end; ++i)
    csum ^= buf[i];
  return csum;
}

void IRAM_ATTR isrHall() {
  uint32_t now = micros();
  portENTER_CRITICAL_ISR(&hall_mux);
  uint32_t dt = now - hall.last_pulse;
  if (dt >= HALL_DEBOUNCE_US) {
    // Ghi tường minh thay vì ++: toán tử ++ trên biến volatile đã bị loại bỏ
    // trong C++20 và gcc cảnh báo. Hai lệnh đọc-ghi này nằm trong critical
    // section nên vẫn nguyên tử với phía đọc.
    hall.pulse_cnt = hall.pulse_cnt + 1;
    if (hall.last_pulse != 0) {
      hall.period_us = dt;
      hall.sequence = hall.sequence + 1;
    }
    hall.last_pulse = now;
  }
  portEXIT_CRITICAL_ISR(&hall_mux);
}

// ============================================================================
// [6] XỬ LÝ LÕI TÍNH TOÁN & GIAO TIẾP
// ============================================================================

// --- 6.1. ĐỌC TỐC ĐỘ ---
void calcSpeed() {
  uint32_t period = 0, last_p = 0, seq = 0;
  static uint32_t last_seq = 0;

  portENTER_CRITICAL(&hall_mux);
  last_p = hall.last_pulse;
  period = hall.period_us;
  seq = hall.sequence;
  portEXIT_CRITICAL(&hall_mux);

  uint32_t now = micros();

  // 1. Có chu kỳ mới
  if (seq != last_seq && period > 0) {
    last_seq = seq;

    // Chặn dưới để bỏ xung nhiễu/khiếp khuyết (period rất nhỏ là nhiễu, không
    // phải tốc độ cao). KHÔNG dùng MIN_HALL_PERIOD_US ở đây: hằng đó là
    // "chu kỳ ứng với 20 km/h", dùng làm điều kiện nhận thì mọi tốc độ
    // trên 20 km/h đều bị bỏ qua và tốc độ đo được đứng yên ở giá trị
    // cuối cùng hợp lệ. Thay vào đó nhận mọi chu kỳ rồi KẸP tốc độ.
    if (period >= MIN_HALL_PERIOD_US_NOISE) {
      const float raw_speed = PERIOD_TO_KMH_FACTOR / static_cast<float>(period);
      const float safe_speed = constrain(raw_speed, 0.0f, MAX_VALID_SPEED_KMH);

      float adaptive_alpha;
      if (safe_speed < 3.0f)
        adaptive_alpha = 0.60f;
      else if (safe_speed < 8.0f)
        adaptive_alpha = 0.40f;
      else
        adaptive_alpha = 0.30f;

      hall.smooth_speed_kmh = adaptive_alpha * safe_speed + (1.0f - adaptive_alpha) * hall.smooth_speed_kmh;
      car.cur_spd = hall.smooth_speed_kmh;

      hall.proc_pulse_us = last_p;
      hall.valid_period = period;
      hall.saved_spd = car.cur_spd;
    }
  }

  // 2. Nội suy khi chờ xung
  if (hall.proc_pulse_us != 0 && hall.valid_period > 0) {
    uint32_t elapsed = now - hall.proc_pulse_us;
    uint32_t hold_us = constrain(hall.valid_period + (hall.valid_period / 2), 20000UL, 500000UL);
    uint32_t decay_us = constrain(hall.valid_period * 2UL, 50000UL, 1000000UL);

    if (elapsed <= hold_us) {
      car.cur_spd = hall.saved_spd;
    } else {
      float decay = constrain(static_cast<float>(elapsed - hold_us) / static_cast<float>(decay_us), 0.0f, 1.0f);
      car.cur_spd = hall.saved_spd * (1.0f - decay);

      if (decay >= 1.0f) {
        car.cur_spd = 0.0f;
        hall.smooth_speed_kmh = 0.0f;
      }
    }
  } else {
    car.cur_spd = 0.0f;
    hall.smooth_speed_kmh = 0.0f;
  }

  // 3. Timeout chặn cuối
  uint32_t adaptive_timeout_us = 1000000;
  if (hall.valid_period > 0) {
    adaptive_timeout_us = constrain(hall.valid_period * 4, 100000UL, 1000000UL);
  }

  if (last_p == 0 || (now - last_p) >= adaptive_timeout_us) {
    car.cur_spd = 0.0f;
    hall.smooth_speed_kmh = 0.0f;
  }

  car.cur_spd = constrain(car.cur_spd, 0.0f, MAX_VALID_SPEED_KMH);
}

// --- 6.2. MÁY TRẠNG THÁI ĐỌC UART ---
void readUART() {
  uint32_t start_us = micros();

  if (rx_state != RxState::WAIT_H1 && (start_us - last_rx_us) > FRAME_TIMEOUT_US) {
    rx_state = RxState::WAIT_H1;
    rx_idx = 0;
  }

  uint16_t count = 0;

  while (Serial.available() > 0 && count < MAX_RX_PER_LOOP) {
    if ((micros() - start_us) >= CPU_RX_BUDGET_US)
      break;

    count++;
    uint8_t b = Serial.read();
    last_rx_us = micros();

    switch (rx_state) {
    case RxState::WAIT_H1:
      if (b == HDR_RX1) {
        rx_buf[0] = b;
        rx_idx = 1;
        rx_state = RxState::WAIT_H2;
      }
      break;

    case RxState::WAIT_H2:
      if (b == HDR_RX2) {
        rx_buf[1] = b;
        rx_idx = 2;
        rx_state = RxState::READ_PAYLOAD;
      } else if (b == HDR_RX1) {
        rx_buf[0] = b;
        rx_idx = 1;
      } else {
        rx_state = RxState::WAIT_H1;
      }
      break;

    case RxState::READ_PAYLOAD:
      if (rx_idx < RX_LEN)
        rx_buf[rx_idx++] = b;

      if (rx_idx >= RX_LEN) {
        if (calcXor(rx_buf, 2, IDX_CSUM - 1) == rx_buf[IDX_CSUM]) {
          car.raw_dev = (int16_t)((rx_buf[IDX_LANE_H] << 8) | rx_buf[IDX_LANE_L]);
          car.target_spd = constrain(rx_buf[IDX_SPEED] * 0.1f, 0.0f, MAX_SPEED_KMH);
          car.emg_stop = (rx_buf[IDX_EMG] != 0);
          last_packet_ms = millis();
        }
        rx_state = RxState::WAIT_H1;
        rx_idx = 0;
      }
      break;
    }
  }
}

// --- 6.3. THUẬT TOÁN ĐIỀU KHIỂN (PID) ---
int getBasePWM(float spd) {
  if (spd < 0.5f)
    return ESC_NEUTRAL;
  if (spd < ESC_LUT[2].spd)
    return ESC_MIN_FWD;

  for (size_t i = 0; i + 1 < LUT_SIZE; ++i) {
    if (spd >= ESC_LUT[i].spd && spd < ESC_LUT[i + 1].spd) {
      float dSpd = ESC_LUT[i + 1].spd - ESC_LUT[i].spd;
      if (dSpd <= 0.0f)
        return ESC_LUT[i].pwm;
      float ratio = (spd - ESC_LUT[i].spd) / dSpd;
      return roundf(ESC_LUT[i].pwm + ratio * (ESC_LUT[i + 1].pwm - ESC_LUT[i].pwm));
    }
  }
  return ESC_LUT[LUT_SIZE - 1].pwm;
}

void calcSpeedPID(float dt) {
  if (car.target_spd < 0.5f) {
    pid_speed.integral = pid_speed.prev_err = 0;
    car.esc_cmd = ESC_NEUTRAL;
    return;
  }

  int base_pwm = max(getBasePWM(car.target_spd), ESC_MIN_FWD);
  float err = car.target_spd - car.cur_spd;
  float max_dpwm;
  int margin;

  if (car.cur_spd <= 3.58f) {
    pid_speed.kp = (err < -0.3f) ? 1.5f : 1.2f;
    pid_speed.ki = 0.02f;
    pid_speed.kd = 0.005f;
    max_dpwm = 3.0f;
    margin = 1;
  } else if (car.cur_spd <= 8.0f) {
    pid_speed.kp = 0.5f;
    pid_speed.ki = 0.03f;
    pid_speed.kd = 0.008f;
    max_dpwm = 4.0f;
    margin = 5;
  } else if (car.cur_spd <= 12.0f) {
    pid_speed.kp = 1.0f;
    pid_speed.ki = 0.04f;
    pid_speed.kd = 0.01f;
    max_dpwm = 5.0f;
    margin = 8;
  } else {
    pid_speed.kp = 1.2f;
    pid_speed.ki = 0.04f;
    pid_speed.kd = 0.01f;
    max_dpwm = 6.0f;
    margin = 10;
  }

  float P = pid_speed.kp * err;

  if (car.cur_spd > car.target_spd)
    pid_speed.integral *= 0.2f;
  else
    pid_speed.integral += err * dt;
  pid_speed.integral = constrain(pid_speed.integral, -10.0f, 10.0f);

  float I = pid_speed.ki * pid_speed.integral;
  float D = pid_speed.kd * (err - pid_speed.prev_err) / dt;
  pid_speed.prev_err = err;

  float total = constrain(P + I + D, -max_dpwm, max_dpwm);
  car.esc_cmd = constrain(car.esc_cmd + (int)roundf(total), ESC_NEUTRAL, ESC_MAX_FWD);

  if (err > 0.0f) {
    car.esc_cmd = constrain(car.esc_cmd, ESC_MIN_FWD, base_pwm + margin);
  } else if (err < -0.3f && car.esc_cmd > base_pwm) {
    car.esc_cmd = base_pwm;
  }
}

void calcSteerPID(float dt) {
  car.smooth_dev = ALPHA_STEER * car.raw_dev + (1.0f - ALPHA_STEER) * car.smooth_dev;

  if (car.cur_spd < 10.0f) {
    pid_steer.kp = 1.5f;
    pid_steer.ki = 0.08f;
    pid_steer.kd = 0.6f;
  } else if (car.cur_spd < 25.0f) {
    pid_steer.kp = 1.0f;
    pid_steer.ki = 0.05f;
    pid_steer.kd = 0.9f;
  } else {
    pid_steer.kp = 0.6f;
    pid_steer.ki = 0.02f;
    pid_steer.kd = 1.3f;
  }

  float target_angle = STEER_CENTER;
  float abs_dev = fabsf(car.smooth_dev);

  if (abs_dev > CAM_DEADZONE) {
    float norm_dev = constrain(abs_dev - CAM_DEADZONE, 0.0f, CAM_MAX_DEV - CAM_DEADZONE);
    float offset = mapF(norm_dev, 0.0f, CAM_MAX_DEV - CAM_DEADZONE, 0.0f, STEER_MAX - STEER_CENTER);
    target_angle = (car.smooth_dev < 0.0f) ? (STEER_CENTER + offset) : (STEER_CENTER - offset);
  }

  float err = target_angle - STEER_CENTER;
  float P = pid_steer.kp * err;

  if (fabsf(err) < 1.0f)
    pid_steer.integral *= 0.5f;
  else
    pid_steer.integral += err * dt;
  pid_steer.integral = constrain(pid_steer.integral, -15.0f, 15.0f);

  float I = pid_steer.ki * pid_steer.integral;
  float D = pid_steer.kd * (err - pid_steer.prev_err) / dt;
  pid_steer.prev_err = err;

  car.steer_cmd = (int)roundf(constrain(STEER_CENTER + P + I + D, (float)STEER_MIN, (float)STEER_MAX));
}

void runPID(uint32_t now) {
  if (!pid_timer_init) {
    next_pid_us = now + PID_DT_US;
    pid_timer_init = true;
    return;
  }

  if ((int32_t)(now - next_pid_us) < 0)
    return;

  next_pid_us += PID_DT_US;
  if ((int32_t)(now - next_pid_us) >= 0)
    next_pid_us = now + PID_DT_US;

  constexpr float DT_SEC = PID_DT_US * 1e-6f;

  if (!car.emg_stop) {
    calcSpeedPID(DT_SEC);
    if (car.esc_cmd != last_esc) {
      motor_esc.write(car.esc_cmd);
      last_esc = car.esc_cmd;
    }
  }

  calcSteerPID(DT_SEC);
  if (car.steer_cmd != last_steer) {
    servo_steer.write(car.steer_cmd);
    last_steer = car.steer_cmd;
  }
}

// ============================================================================
// [7] CƠ CHẾ BẢO VỆ (SAFETY / BRAKING)
// ============================================================================
void processBrake() {
  static uint8_t phase = 0;
  static uint32_t start_ms = 0;
  uint32_t now = millis();

  if (!car.braking) {
    car.braking = true;
    phase = 0;
    start_ms = now;
  }

  if (phase == 0) {
    if (last_esc != ESC_BRAKE) {
      motor_esc.write(ESC_BRAKE);
      last_esc = ESC_BRAKE;
    }
    if (now - start_ms >= BRAKE_HOLD_MS) {
      phase = 1;
      start_ms = now;
    }
  } else {
    if (last_esc != ESC_NEUTRAL) {
      motor_esc.write(ESC_NEUTRAL);
      last_esc = ESC_NEUTRAL;
    }
    car.esc_cmd = ESC_NEUTRAL;
    pid_speed.integral = pid_speed.prev_err = 0;
    car.braking = false;
  }
}

void checkSafety() {
  if (millis() - last_packet_ms > WDOG_TIMEOUT_MS) {
    car.target_spd = 0;
    car.raw_dev = 0;
    car.emg_stop = true;
  }

  if (!car.emg_stop) {
    if (car.braking) {
      car.braking = false;
      if (last_esc != ESC_NEUTRAL) {
        motor_esc.write(ESC_NEUTRAL);
        last_esc = ESC_NEUTRAL;
      }
      car.esc_cmd = ESC_NEUTRAL;
      pid_speed.integral = pid_speed.prev_err = 0;
    }
    return;
  }

  // Đang phanh thì tiếp tục phanh.
  if (car.braking) {
    processBrake();
    return;
  }

  // Xe vẫn còn lăn (sau khi đổi số mo phanh điện tử) thì phanh tiếp.
  // Bản cũ chỉ phanh MỘT lần rồi giữ cờ emg_started = true vĩnh viễn: xe trượt
  // tiếp và có thể còn lao vào vật cản dù vẫn ở trạng thái EMERGENCY_STOP.
  if (car.cur_spd > STOPPED_KMH) {
    processBrake();
    return;
  }

  // Đã dừng hẳn: giữ số mo.
  if (last_esc != ESC_NEUTRAL) {
    motor_esc.write(ESC_NEUTRAL);
    last_esc = ESC_NEUTRAL;
  }
  car.esc_cmd = ESC_NEUTRAL;
}

// ============================================================================
// [8] ĐIỀU KHIỂN NGOẠI VI & TELEMETRY
// ============================================================================
void updateLights() {
  static uint32_t last_blink = 0;
  static bool led_on = false;
  uint32_t now = millis();

  if (now - last_blink >= BLINK_DT_MS) {
    led_on = !led_on;
    last_blink = now;
  }

  if (fabsf(car.smooth_dev) < BLINK_THRESH) {
    digitalWrite(PIN_TURN_L, LOW);
    digitalWrite(PIN_TURN_R, LOW);
  } else if (car.smooth_dev < -BLINK_THRESH) {
    digitalWrite(PIN_TURN_L, led_on);
    digitalWrite(PIN_TURN_R, LOW);
  } else {
    digitalWrite(PIN_TURN_L, LOW);
    digitalWrite(PIN_TURN_R, led_on);
  }

  digitalWrite(PIN_BRAKE, (car.target_spd < 0.5f || car.emg_stop || car.braking));
}

void sendTelemetry() {
  uint8_t tx_buf[TX_LEN] = {HDR_TX1, HDR_TX2, 0, 0, 0, 0, 0};
  memcpy(&tx_buf[2], &car.cur_spd, sizeof(float));
  tx_buf[6] = calcXor(tx_buf, 2, 5);

  if (Serial.availableForWrite() >= TX_LEN)
    Serial.write(tx_buf, TX_LEN);
}

// ============================================================================
// [9] SETUP & LOOP
// ============================================================================
void setup() {
  Serial.setRxBufferSize(RX_BUF_SIZE);
  Serial.setTxBufferSize(TX_BUF_SIZE);
  Serial.begin(UART_BAUD);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  pinMode(PIN_HALL, INPUT_PULLUP);
  pinMode(PIN_TURN_L, OUTPUT);
  pinMode(PIN_TURN_R, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_TURN_L, LOW);
  digitalWrite(PIN_TURN_R, LOW);
  digitalWrite(PIN_BRAKE, LOW);

  servo_steer.setPeriodHertz(50);
  servo_steer.attach(PIN_STEER, 500, 2400);
  servo_steer.write(STEER_CENTER);

  motor_esc.attach(PIN_ESC, 1000, 2000);
  motor_esc.write(ESC_NEUTRAL);

  attachInterrupt(digitalPinToInterrupt(PIN_HALL), isrHall, RISING);

  last_packet_ms = millis();
  next_pid_us = micros() + PID_DT_US;
  pid_timer_init = true;
}

void loop() {
  // 1. Quét đọc UART (Non-blocking)
  readUART();

  // 2. Đo vận tốc Hall
  calcSpeed();

  // 3. Đánh giá an toàn & phanh
  checkSafety();

  // 4. PID nhịp 100Hz
  runPID(micros());

  // 5. Cập nhật đèn
  updateLights();

  // 6. Phản hồi Telemetry 50Hz (Chỉ gửi frame nhị phân, không log text)
  static uint32_t last_tx = millis();
  if (millis() - last_tx >= TELEM_DT_MS) {
    sendTelemetry();
    last_tx = millis();
  }
}
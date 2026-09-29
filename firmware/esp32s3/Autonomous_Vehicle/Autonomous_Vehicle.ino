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
constexpr uint8_t PIN_STEER = 13;  // Chân PWM xuất ra Servo bẻ lái
constexpr uint8_t PIN_HALL = 23;   // Chân ngắt đọc xung Cảm biến Hall đo tốc độ
constexpr uint8_t PIN_TURN_L = 21; // Đèn LED xi-nhan trái
constexpr uint8_t PIN_TURN_R = 18; // Đèn LED xi-nhan phải (Tránh chân 19 là USB_D- của ESP32-S3)
constexpr uint8_t PIN_BRAKE = 15;  // Đèn LED phanh đít
constexpr uint8_t PIN_ESC = 14;    // Chân PWM xuất ra Động cơ (ESC)

// ============================================================================
    // [2] THÔNG SỐ VẬT LÝ & ĐIỀU KHIỂN (PARAMETERS)
    // ============================================================================
    // --- Hệ thống lái (Steering) ---
    constexpr int STEER_CENTER = 90; // Góc Servo khi xe đi thẳng
constexpr int STEER_MIN = 45;        // Giới hạn góc Servo rẽ phải tối đa
constexpr int STEER_MAX = 135;       // Giới hạn góc Servo rẽ trái tối đa

constexpr float CAM_DEADZONE =
    10.0f; // Vùng chết (pixel): Lệch dưới mức này xem như đi thẳng
constexpr float CAM_MAX_DEV =
    50.0f; // Độ lệch quỹ đạo tối đa (pixel) để quy đổi ra góc lái
constexpr float BLINK_THRESH =
    10.0f; // Ngưỡng lệch làn (pixel) để tự động bật xi-nhan

// --- Động cơ (Speed/ESC) ---
constexpr int ESC_NEUTRAL = 90;  // Xung PWM để ESC dừng động cơ (Số Mo)
constexpr int ESC_MIN_FWD = 95;  // Xung PWM tối thiểu để xe bắt đầu nhích tới
constexpr int ESC_MAX_FWD = 180; // Xung PWM tối đa cho chiều tiến
constexpr int ESC_BRAKE = 20;    // Xung PWM kéo lùi để hãm phanh điện từ

constexpr float MAX_SPEED_KMH =
    15.0f; // Giới hạn tốc độ mục tiêu an toàn từ MiniPC
constexpr float MAX_VALID_SPEED_KMH =
    20.0f; // Ngưỡng chặn nhiễu sinh tốc độ ảo vượt 20km/h

// --- Toán học tính vận tốc (Tính sẵn tại Compile-time để MCU chạy siêu nhẹ)
// ---
constexpr float WHEEL_DIA_M = 0.090f; // Đường kính bánh xe (90mm = 0.09m)
constexpr float WHEEL_CIRCUMF = PI * WHEEL_DIA_M; // Chu vi bánh xe (m)
constexpr float GEAR_RATIO = 37.0f / 13.0f;       // Tỷ số truyền vi sai
constexpr uint8_t PULSES_PER_REV = 4; // Số lượng xung Hall / 1 vòng quay bánh

// Hằng số quy đổi từ Chu kỳ (us) sang Tốc độ (km/h) = (3.6 * 1,000,000 * Chu
// vi) / (Xung * Tỷ số)
constexpr float PERIOD_TO_KMH_FACTOR =
    (3600000.0f * WHEEL_CIRCUMF) / (PULSES_PER_REV * GEAR_RATIO);
// Chu kỳ xung nhỏ nhất hợp lý (Giới hạn vật lý khi xe chạy 20km/h để chặn xung
// nhiễu quá sát)
constexpr uint32_t MIN_HALL_PERIOD_US =
    (uint32_t)(PERIOD_TO_KMH_FACTOR / MAX_VALID_SPEED_KMH);

// --- Bộ lọc và Thời gian định thời (Timing & Filters) ---
constexpr uint32_t PID_DT_US = 10000; // 100Hz: Chu kỳ định thời thuật toán PID
constexpr uint32_t TELEM_DT_MS =
    20; // 50Hz: Chu kỳ gửi phản hồi trạng thái về MiniPC
constexpr uint32_t WDOG_TIMEOUT_MS =
    500; // 0.5s: Nếu mất kết nối PC quá nửa giây -> Dừng xe
constexpr uint32_t BLINK_DT_MS = 500; // 0.5s: Tốc độ nháy đèn xi-nhan

constexpr uint32_t HALL_DEBOUNCE_US =
    1000; // 1ms: Lọc nhiễu công tắc cảm biến Hall
constexpr uint32_t BRAKE_HOLD_MS =
    200; // 200ms: Thời gian giữ lệnh lùi để hãm xe phanh
constexpr uint32_t FRAME_TIMEOUT_US =
    10000; // 10ms: Thời gian chờ tối đa cho 1 frame UART bị đứt đoạn

constexpr float ALPHA_STEER =
    0.4f; // Hệ số EMA làm mượt góc lái (0->1, càng nhỏ càng mượt)

// --- Giao tiếp (UART Protocol) ---
constexpr uint32_t UART_BAUD = 230400; // Tốc độ Baud giao tiếp MiniPC
constexpr size_t RX_BUF_SIZE =
    4096; // Kích thước Buffer nhận (Rộng để chống tràn burst data)
constexpr size_t TX_BUF_SIZE = 512; // Kích thước Buffer gửi

constexpr uint8_t HDR_RX1 = 0xAB, HDR_RX2 = 0xCD; // Byte đồng bộ nhận lệnh
constexpr uint8_t HDR_TX1 = 0xDC, HDR_TX2 = 0xBA; // Byte đồng bộ gửi Telemetry
constexpr uint8_t RX_LEN = 11; // Kích thước chuẩn gói tin nhận
constexpr uint8_t TX_LEN = 7;  // Kích thước chuẩn gói tin gửi

constexpr uint32_t CPU_RX_BUDGET_US =
    800; // Ngân sách thời gian tối đa để đọc UART (800us/vòng)
constexpr uint16_t MAX_RX_PER_LOOP =
    2048; // Giới hạn đọc byte tối đa 1 lượt chống nghẽn vòng lặp

// Vị trí mảng của gói tin RX
constexpr uint8_t IDX_LANE_H = 2, IDX_LANE_L = 3;
constexpr uint8_t IDX_SPEED = 4, IDX_EMG = 5, IDX_CSUM = 10;

// ============================================================================
// [3] CẤU TRÚC DỮ LIỆU (DATA STRUCTURES)
// ============================================================================
// Trạng thái tổng quát của xe
struct CarState {
  float target_spd = 0.0f;      // Vận tốc mong muốn từ PC
  float cur_spd = 0.0f;         // Vận tốc thực tế đo được
  int16_t raw_dev = 0;          // Độ lệch làn thô (pixel)
  float smooth_dev = 0.0f;      // Độ lệch làn đã lọc mượt EMA
  int steer_cmd = STEER_CENTER; // Góc ra Servo bẻ lái
  int esc_cmd = ESC_NEUTRAL;    // Xung ra Động cơ ESC
  bool emg_stop = false;        // Cờ dừng khẩn cấp
  bool braking = false;         // Đang trong quá trình phanh
};

// Cấu trúc bộ thông số PID
struct PidState {
  float kp = 0.0f, ki = 0.0f, kd = 0.0f;
  float integral = 0.0f; // Khâu I (Cộng dồn sai số)
  float prev_err = 0.0f; // Khâu D (Lỗi vòng lặp trước)
};

// Trạng thái Cảm biến Hall
struct HallState {
  volatile uint32_t last_pulse = 0; // Thời điểm nhận xung cuối (Micros)
  volatile uint32_t period_us = 0;  // Khoảng thời gian giữa 2 xung gần nhất
  volatile uint32_t pulse_cnt = 0;  // Tổng số đếm xung
  volatile uint32_t sequence = 0;   // Đánh dấu ID chu kỳ (Tăng khi có xung mới)

  uint32_t proc_pulse_us = 0;    // Thời điểm xử lý xung cuối cùng trong loop
  uint32_t valid_period = 0;     // Chu kỳ hợp lệ gần nhất
  float smooth_speed_kmh = 0.0f; // Tốc độ đã được lọc mượt
  float saved_spd = 0.0f;        // Tốc độ lưu nháp để nội suy giảm dần
};

// Máy trạng thái đọc UART
enum class RxState : uint8_t { WAIT_H1, WAIT_H2, READ_PAYLOAD };

// ============================================================================
// [4] BIẾN TOÀN CỤC (GLOBALS)
// ============================================================================
Servo servo_steer, motor_esc;

CarState car;
PidState pid_steer = {1.5f, 0.08f, 0.6f, 0.0f,
                      0.0f}; // Khởi tạo PID lái mặc định
PidState pid_speed;
HallState hall;

// Khóa Spinlock (MUX) bảo vệ chống xung đột bộ nhớ giữa 2 nhân CPU (ISR vs
// Loop)
portMUX_TYPE hall_mux = portMUX_INITIALIZER_UNLOCKED;

// Quản lý UART RX
RxState rx_state = RxState::WAIT_H1;
uint8_t rx_buf[RX_LEN] = {};
uint8_t rx_idx = 0;
uint32_t last_rx_us = 0;
uint32_t last_packet_ms = 0; // Lưu thời điểm nhận lệnh để Watchdog canh trừng

// Quản lý PID Scheduler
uint32_t next_pid_us = 0;
bool pid_timer_init = false;
bool emg_started = false;

int last_esc = ESC_NEUTRAL;
int last_steer = STEER_CENTER;

// Bảng tra cứu tốc độ tĩnh (ESC Lookup Table)
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
// Hàm quy đổi dải số có chặn giới hạn an toàn để không vượt biên
inline float mapF(float x, float in_min, float in_max, float out_min,
                  float out_max) {
  if (in_max == in_min)
    return out_min;                 // Tránh lỗi chia cho 0
  x = constrain(x, in_min, in_max); // Cắt phần vượt giới hạn
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

// Hàm tính Checksum XOR đơn giản
inline uint8_t calcXor(const uint8_t *buf, uint8_t start, uint8_t end) {
  uint8_t csum = 0;
  for (uint8_t i = start; i <= end; ++i)
    csum ^= buf[i];
  return csum;
}

// Hàm NGẮT CẢM BIẾN HALL: Chạy cực nhanh mỗi khi bánh xe quay qua nam châm
void IRAM_ATTR isrHall() {
  uint32_t now = micros();

  portENTER_CRITICAL_ISR(
      &hall_mux); // Khóa nhân CPU còn lại để ghi biến an toàn
  uint32_t dt = now - hall.last_pulse;
  if (dt >= HALL_DEBOUNCE_US) { // Loại bỏ nhiễu rung cơ học (<1ms)
    hall.pulse_cnt++;
    if (hall.last_pulse != 0) {
      hall.period_us = dt; // Ghi lại chu kỳ
      hall.sequence++;     // Tăng ID để vòng lặp biết có chu kỳ mới
    }
    hall.last_pulse = now;
  }
  portEXIT_CRITICAL_ISR(&hall_mux); // Mở khóa CPU
}

// ============================================================================
// [6] XỬ LÝ LÕI TÍNH TOÁN & GIAO TIẾP
// ============================================================================

// --- 6.1. ĐỌC TỐC ĐỘ (SPEED CALCULATION - TỐI ƯU XE CHẠY CHẬM) ---
void calcSpeed() {
  uint32_t period = 0, last_p = 0, seq = 0;
  static uint32_t last_seq = 0;

  // Lấy bản sao dữ liệu từ ngắt ra ngoài một cách an toàn
  portENTER_CRITICAL(&hall_mux);
  last_p = hall.last_pulse;
  period = hall.period_us;
  seq = hall.sequence;
  portEXIT_CRITICAL(&hall_mux);

  uint32_t now = micros();

  // 1. NẾU CÓ CHU KỲ MỚI (Xe đang lăn bánh liên tục)
  if (seq != last_seq && period > 0) {
    last_seq = seq;

    // Bỏ qua các xung nhiễu sinh tốc độ ảo >20km/h
    if (period >= MIN_HALL_PERIOD_US) {
      // Tính vận tốc Km/h từ chu kỳ, dùng hằng số ma thuật (Chỉ tốn 1 phép
      // chia)
      const float raw_speed = PERIOD_TO_KMH_FACTOR / static_cast<float>(period);
      const float safe_speed = constrain(raw_speed, 0.0f, MAX_VALID_SPEED_KMH);

      // [LỌC NHIỄU THÍCH NGHI - Adaptive EMA]
      // Đi chậm -> lấy alpha lớn để phản ứng lẹ. Đi nhanh -> lấy alpha nhỏ để
      // xe êm.
      float adaptive_alpha;
      if (safe_speed < 3.0f)
        adaptive_alpha = 0.60f;
      else if (safe_speed < 8.0f)
        adaptive_alpha = 0.40f;
      else
        adaptive_alpha = 0.30f;

      hall.smooth_speed_kmh = adaptive_alpha * safe_speed +
                              (1.0f - adaptive_alpha) * hall.smooth_speed_kmh;
      car.cur_spd = hall.smooth_speed_kmh;

      // Lưu dữ liệu để lát nữa nội suy nếu xe chạy chậm dần
      hall.proc_pulse_us = last_p;
      hall.valid_period = period;
      hall.saved_spd = car.cur_spd;
    }
  }

  // 2. NẾU ĐANG TRONG KHOẢNG CHỜ XUNG TIẾP THEO (Nội suy giảm tốc độ êm ái)
  if (hall.proc_pulse_us != 0 && hall.valid_period > 0) {
    uint32_t elapsed = now - hall.proc_pulse_us;

    // Khoảng chờ an toàn: Giữ tốc độ nguyên si trong tối đa 1.5 lần chu kỳ
    // trước (Max 0.5s)
    uint32_t hold_us = constrain(hall.valid_period + (hall.valid_period / 2),
                                 20000UL, 500000UL);
    // Trục phân rã: Tốc độ sẽ tuột dần về 0 trong 2 lần chu kỳ (Max 1s)
    uint32_t decay_us = constrain(hall.valid_period * 2UL, 50000UL, 1000000UL);

    if (elapsed <= hold_us) {
      car.cur_spd = hall.saved_spd; // Vẫn đang bình thường
    } else {
      // Xe đang chạy chậm lại -> Giảm tốc độ tuyến tính để PID không bị giật
      // cục
      float decay = constrain(static_cast<float>(elapsed - hold_us) /
                                  static_cast<float>(decay_us),
                              0.0f, 1.0f);
      car.cur_spd = hall.saved_spd * (1.0f - decay);

      if (decay >=
          1.0f) { // Thời gian trôi qua đã đủ lâu -> Chắc chắn xe đã dừng
        car.cur_spd = 0.0f;
        hall.smooth_speed_kmh = 0.0f;
      }
    }
  } else {
    car.cur_spd = 0.0f;
    hall.smooth_speed_kmh = 0.0f;
  }

  // 3. CHỐT CHẶN CUỐI THÍCH NGHI (Adaptive Timeout)
  // Tùy theo xe vừa đi chậm hay nhanh, ta cho phép nó chờ xung tiếp theo tối đa
  // 1 giây
  uint32_t adaptive_timeout_us = 1000000;
  if (hall.valid_period > 0) {
    adaptive_timeout_us = constrain(hall.valid_period * 4, 100000UL, 1000000UL);
  }

  // Nếu quá ngưỡng chờ cho phép -> Đưa hẳn về 0
  if (last_p == 0 || (now - last_p) >= adaptive_timeout_us) {
    car.cur_spd = 0.0f;
    hall.smooth_speed_kmh = 0.0f;
  }

  // Kẹp thêm 1 lần cuối cho an toàn
  car.cur_spd = constrain(car.cur_spd, 0.0f, MAX_VALID_SPEED_KMH);
}

// --- 6.2. MÁY TRẠNG THÁI ĐỌC UART (UART RX) ---
void readUART() {
  uint32_t start_us = micros();

  // Hủy và khởi động lại luồng nếu gói tin bị đứt đoạn quá 10ms
  if (rx_state != RxState::WAIT_H1 &&
      (start_us - last_rx_us) > FRAME_TIMEOUT_US) {
    rx_state = RxState::WAIT_H1;
    rx_idx = 0;
  }

  uint16_t count = 0;

  // Đọc liên tục với điều kiện: Có data VÀ Chưa tới ngưỡng chống nghẽn vòng lặp
  // VÀ Vẫn còn ngân sách CPU
  while (Serial.available() > 0 && count < MAX_RX_PER_LOOP) {
    if ((micros() - start_us) >= CPU_RX_BUDGET_US)
      break; // Hết thời gian cho UART -> Thoát đi chạy PID

    count++;
    uint8_t b = Serial.read();
    last_rx_us = micros();

    switch (rx_state) {
    case RxState::WAIT_H1: // Chờ Header 1 (AB)
      if (b == HDR_RX1) {
        rx_buf[0] = b;
        rx_idx = 1;
        rx_state = RxState::WAIT_H2;
      }
      break;

    case RxState::WAIT_H2: // Chờ Header 2 (CD)
      if (b == HDR_RX2) {
        rx_buf[1] = b;
        rx_idx = 2;
        rx_state = RxState::READ_PAYLOAD;
      } else if (b == HDR_RX1) {
        rx_buf[0] = b;
        rx_idx = 1;
      } // Rơi nhầm, bắt đầu lại
      else {
        rx_state = RxState::WAIT_H1;
      }
      break;

    case RxState::READ_PAYLOAD: // Đọc tiếp 9 byte phần thân
      if (rx_idx < RX_LEN)
        rx_buf[rx_idx++] = b;

      if (rx_idx >= RX_LEN) {
        // Nhận đủ 11 bytes, kiểm tra Checksum XOR
        if (calcXor(rx_buf, 2, IDX_CSUM - 1) == rx_buf[IDX_CSUM]) {
          // Trùng khớp -> Cập nhật trực tiếp vào trạng thái chung của xe
          // (Zero-Latency)
          car.raw_dev = (rx_buf[IDX_LANE_H] << 8) | rx_buf[IDX_LANE_L];
          car.target_spd =
              constrain(rx_buf[IDX_SPEED] * 0.1f, 0.0f, MAX_SPEED_KMH);
          car.emg_stop = (rx_buf[IDX_EMG] != 0);
          last_packet_ms = millis(); // Nuôi chó canh chừng (Watchdog)
        }
        rx_state = RxState::WAIT_H1; // Đợi gói mới
        rx_idx = 0;
      }
      break;
    }
  }
}

// --- 6.3. THUẬT TOÁN ĐIỀU KHIỂN (CONTROL PID) ---
// Hàm nội suy: Tính ra xung PWM cơ bản cho ESC từ vận tốc mong muốn
int getBasePWM(float spd) {
  if (spd < 0.5f)
    return ESC_NEUTRAL; // Chậm quá -> Dừng
  if (spd < ESC_LUT[2].spd)
    return ESC_MIN_FWD; // Nhích nhẹ

  // Quét dọc bảng Lookup để tìm mốc chặn gần nhất
  for (size_t i = 0; i + 1 < LUT_SIZE; ++i) {
    if (spd >= ESC_LUT[i].spd && spd < ESC_LUT[i + 1].spd) {
      float dSpd = ESC_LUT[i + 1].spd - ESC_LUT[i].spd;
      if (dSpd <= 0.0f)
        return ESC_LUT[i].pwm;
      float ratio = (spd - ESC_LUT[i].spd) / dSpd;
      return roundf(ESC_LUT[i].pwm +
                    ratio * (ESC_LUT[i + 1].pwm - ESC_LUT[i].pwm));
    }
  }
  return ESC_LUT[LUT_SIZE - 1].pwm; // Vượt bảng -> Trả về cao nhất
}

// PID Tính toán Động cơ
void calcSpeedPID(float dt) {
  if (car.target_spd < 0.5f) { // Nhận lệnh dừng hẳn
    pid_speed.integral = pid_speed.prev_err = 0;
    car.esc_cmd = ESC_NEUTRAL;
    return;
  }

  int base_pwm = max(getBasePWM(car.target_spd), ESC_MIN_FWD);
  float err = car.target_spd - car.cur_spd;
  float max_dpwm;
  int margin;

  // Tự động tinh chỉnh PID (Adaptive) theo dải tốc độ thực tế
  if (car.cur_spd <= 3.58f) {
    pid_speed.kp = (err < -0.3f) ? 1.5f : 1.2f; // Phanh cứng hơn nếu lố tốc
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

  // Chống Windup (Xả khâu I nếu xe chạy trớn quá đà)
  if (car.cur_spd > car.target_spd)
    pid_speed.integral *= 0.2f;
  else
    pid_speed.integral += err * dt;
  pid_speed.integral = constrain(pid_speed.integral, -10.0f, 10.0f);

  float I = pid_speed.ki * pid_speed.integral;
  float D = pid_speed.kd * (err - pid_speed.prev_err) / dt;
  pid_speed.prev_err = err;

  // Cộng dồn lượng bù sai số vào xung ESC hiện tại
  float total = constrain(P + I + D, -max_dpwm, max_dpwm);
  car.esc_cmd =
      constrain(car.esc_cmd + (int)roundf(total), ESC_NEUTRAL, ESC_MAX_FWD);

  // Guard rails: Ép ESC không vọt qua rào an toàn quanh mức Base
  if (err > 0.0f) {
    car.esc_cmd = constrain(car.esc_cmd, ESC_MIN_FWD, base_pwm + margin);
  } else if (err < -0.3f && car.esc_cmd > base_pwm) {
    car.esc_cmd = base_pwm;
  }
}

// PID Tính toán Đánh Lái
void calcSteerPID(float dt) {
  // Lọc nhiễu độ lệch quỹ đạo từ MiniPC bằng bộ lọc Mũ EMA
  car.smooth_dev =
      ALPHA_STEER * car.raw_dev + (1.0f - ALPHA_STEER) * car.smooth_dev;

  // Tốc độ xe càng nhanh, vô-lăng càng đầm (Kp nhỏ lại) tránh lật xe
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

  // Nếu xe lệch ngoài vùng "chết" an toàn (Deadzone) thì mới bẻ lái
  if (abs_dev > CAM_DEADZONE) {
    float norm_dev =
        constrain(abs_dev - CAM_DEADZONE, 0.0f, CAM_MAX_DEV - CAM_DEADZONE);
    float offset = mapF(norm_dev, 0.0f, CAM_MAX_DEV - CAM_DEADZONE, 0.0f,
                        STEER_MAX - STEER_CENTER);
    target_angle = (car.smooth_dev < 0.0f) ? (STEER_CENTER + offset)
                                           : (STEER_CENTER - offset);
  }

  float err = target_angle - STEER_CENTER;
  float P = pid_steer.kp * err;

  // Tránh cộng dồn khâu I khi đang đi đường thẳng tắp
  if (fabsf(err) < 1.0f)
    pid_steer.integral *= 0.5f;
  else
    pid_steer.integral += err * dt;
  pid_steer.integral = constrain(pid_steer.integral, -15.0f, 15.0f);

  float I = pid_steer.ki * pid_steer.integral;
  float D = pid_steer.kd * (err - pid_steer.prev_err) / dt;
  pid_steer.prev_err = err;

  // Tính góc Servo cuối cùng và kẹp góc vật lý
  car.steer_cmd = (int)roundf(
      constrain(STEER_CENTER + P + I + D, (float)STEER_MIN, (float)STEER_MAX));
}

// BỘ ĐỊNH THỜI VÒNG LẶP ĐIỀU KHIỂN (SCHEDULER 100Hz CỐ ĐỊNH)
void runPID(uint32_t now) {
  if (!pid_timer_init) {
    next_pid_us = now + PID_DT_US;
    pid_timer_init = true;
    return;
  }

  // Kiểm tra xem đã tới giờ chạy PID chưa (Ép kiểu giải quyết tràn bộ đếm
  // Timer)
  if ((int32_t)(now - next_pid_us) < 0)
    return;

  next_pid_us += PID_DT_US;
  // Bù sai số trượt (Drift) nếu hệ thống bị trễ 1 nhịp
  if ((int32_t)(now - next_pid_us) >= 0)
    next_pid_us = now + PID_DT_US;

  constexpr float DT_SEC =
      PID_DT_US * 1e-6f; // Cố định dt = 0.01 giây cho toán học PID chuẩn xác

  // Nếu không phải phanh khẩn cấp -> Chạy PID tốc độ
  if (!car.emg_stop) {
    calcSpeedPID(DT_SEC);
    if (car.esc_cmd != last_esc) {
      motor_esc.write(car.esc_cmd);
      last_esc = car.esc_cmd;
    }
  }

  // Luôn luôn chạy đánh lái
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
  static uint8_t phase = 0; // 0: Đạp phanh (Lùi), 1: Thả phanh về Mo
  static uint32_t start_ms = 0;
  uint32_t now = millis();

  // Khởi tạo máy trạng thái phanh
  if (!car.braking) {
    car.braking = true;
    phase = 0;
    start_ms = now;
  }

  if (phase == 0) {
    // Đẩy xung ESC lùi (Brake) để kích hoạt phanh điện từ
    if (last_esc != ESC_BRAKE) {
      motor_esc.write(ESC_BRAKE);
      last_esc = ESC_BRAKE;
    }
    if (now - start_ms >= BRAKE_HOLD_MS) {
      phase = 1;
      start_ms = now;
    }
  } else {
    // Hết thời gian giữ phanh, trả về Neutral
    if (last_esc != ESC_NEUTRAL) {
      motor_esc.write(ESC_NEUTRAL);
      last_esc = ESC_NEUTRAL;
    }
    car.esc_cmd = ESC_NEUTRAL;
    pid_speed.integral = pid_speed.prev_err = 0;
    car.braking = false; // Báo hiệu phanh xong
  }
}

void checkSafety() {
  // 1. Kiểm tra mạch máu (Watchdog) - Nếu quá nửa giây không có lệnh từ PC ->
  // Báo còi khẩn cấp
  if (millis() - last_packet_ms > WDOG_TIMEOUT_MS) {
    car.target_spd = 0;
    car.raw_dev = 0;
    car.emg_stop = true;
  }

  if (!car.emg_stop) {
    emg_started = false;
    return;
  }

  // 2. Kích hoạt cờ Phanh Khẩn Cấp (Chỉ kick 1 lần duy nhất)
  if (!emg_started && !car.braking && car.cur_spd > 0.5f) {
    emg_started = true;
    processBrake();
  }

  // 3. Nuôi máy trạng thái phanh đang chạy
  if (car.braking)
    processBrake();
  else {
    // Nếu đã phanh xong thì khóa cứng ESC ở số Mo
    if (last_esc != ESC_NEUTRAL) {
      motor_esc.write(ESC_NEUTRAL);
      last_esc = ESC_NEUTRAL;
    }
    car.esc_cmd = ESC_NEUTRAL;
  }
}

// ============================================================================
// [8] ĐIỀU KHIỂN ĐÈN & TRUYỀN DỮ LIỆU (PERIPHERALS)
// ============================================================================
void updateLights() {
  static uint32_t last_blink = 0;
  static bool led_on = false;
  uint32_t now = millis();

  // Tính chu kỳ nhấp nháy đèn mỗi nửa giây
  if (now - last_blink >= BLINK_DT_MS) {
    led_on = !led_on;
    last_blink = now;
  }

  // Đèn xi-nhan đánh theo độ lệch làn đường (Chuyển làn / Ôm cua)
  if (fabsf(car.smooth_dev) < BLINK_THRESH) {
    digitalWrite(PIN_TURN_L, LOW);
    digitalWrite(PIN_TURN_R, LOW);
  } else if (car.smooth_dev > BLINK_THRESH) {
    digitalWrite(PIN_TURN_L, led_on);
    digitalWrite(PIN_TURN_R, LOW); // Lệch trái -> Xi nhan Trái
  } else {
    digitalWrite(PIN_TURN_L, LOW);
    digitalWrite(PIN_TURN_R, led_on); // Lệch phải -> Xi nhan Phải
  }

  // Đèn phanh đít bật đỏ khi đang phanh, dừng, hoặc có cờ báo khẩn cấp
  digitalWrite(PIN_BRAKE,
               (car.target_spd < 0.5f || car.emg_stop || car.braking));
}

void sendTelemetry() {
  // Đóng gói mảng byte cần gửi: Header 1, Header 2, Data, Checksum
  uint8_t tx_buf[TX_LEN] = {HDR_TX1, HDR_TX2, 0, 0, 0, 0, 0};
  memcpy(&tx_buf[2], &car.cur_spd,
         4); // Chép số float (4 bytes) tốc độ thẳng vào chuỗi
  tx_buf[6] = calcXor(tx_buf, 2, 5); // Tính mã XOR bảo vệ

  // Chỉ gửi khi buffer phần cứng còn rảnh để tránh làm chẹn CPU
  if (Serial.availableForWrite() >= TX_LEN)
    Serial.write(tx_buf, TX_LEN);
}

// ============================================================================
// [9] HÀM SETUP & MAIN LOOP
// ============================================================================
void setup() {
  // Mở rộng bộ đệm UART ngay từ đầu để hấp thụ dồn ứ dữ liệu (burst) từ MiniPC
  Serial.setRxBufferSize(RX_BUF_SIZE);
  Serial.setTxBufferSize(TX_BUF_SIZE);
  Serial.begin(UART_BAUD);

  // Xin cấp phát Timer nội bộ của ESP32 để phát xung PWM mượt nhất cho
  // Servo/Motor
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);

  // Cài đặt chân I/O
  pinMode(PIN_HALL, INPUT_PULLUP);
  pinMode(PIN_TURN_L, OUTPUT);
  pinMode(PIN_TURN_R, OUTPUT);
  pinMode(PIN_BRAKE, OUTPUT);
  digitalWrite(PIN_TURN_L, LOW);
  digitalWrite(PIN_TURN_R, LOW);
  digitalWrite(PIN_BRAKE, LOW);

  // Khởi tạo thiết bị chấp hành (Trả thẳng lái, Số Mo)
  servo_steer.setPeriodHertz(50);
  servo_steer.attach(PIN_STEER, 500, 2400);
  servo_steer.write(STEER_CENTER);

  motor_esc.attach(PIN_ESC, 1000, 2000);
  motor_esc.write(ESC_NEUTRAL);

  // Bật ngắt phần cứng cho cảm biến nam châm bánh xe
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), isrHall, RISING);

  // Chuẩn bị biến thời gian
  last_packet_ms = millis();
  next_pid_us = micros() + PID_DT_US;
  pid_timer_init = true;
}

void loop() {
  // Bước 1: Quét đọc dữ liệu UART bằng Máy trạng thái (Không chẹn CPU)
  readUART();

  // Bước 2: Đo đạc vận tốc bánh xe (Có thuật toán Adaptive nội suy xe chạy
  // chậm)
  calcSpeed();

  // Bước 3: Đánh giá An toàn mạng lưới & Môi trường để kích hoạt phanh
  checkSafety();

  // Bước 4: Chạy thuật toán PID với nhịp điệu chính xác 100 Hz
  runPID(micros());

  // Bước 5: Cập nhật bóng đèn tín hiệu ngoại vi
  updateLights();

  // Bước 6: Phản hồi thông số về máy tính (50 Hz)
  static uint32_t last_tx = millis();
  if (millis() - last_tx >= TELEM_DT_MS) {
    sendTelemetry();
    last_tx = millis();
  }
}
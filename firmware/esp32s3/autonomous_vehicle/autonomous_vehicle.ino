// ============================================================================
// BỘ ĐIỀU KHIỂN XE TỰ HÀNH ESP32-S3 (AUTONOMOUS VEHICLE)
// Phân hệ: Low-level Controller (Xử lý tín hiệu & Chấp hành)
// ============================================================================

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================================
// [1] CẤU HÌNH CHÂN PHẦN CỨNG (PINS)
// ============================================================================
constexpr uint8_t PIN_STEER = 32;
constexpr uint8_t PIN_ESC = 33;
constexpr uint8_t PIN_TURN_L = 25; 
constexpr uint8_t PIN_TURN_R = 26; 
constexpr uint8_t PIN_BRAKE  = 27; 
constexpr uint8_t PIN_HALL   = 4;  

// ============================================================================
// [2] THÔNG SỐ VẬT LÝ & ĐIỀU KHIỂN (PARAMETERS)
// ============================================================================
constexpr int STEER_CENTER = 90;
constexpr int STEER_MIN = 45;
constexpr int STEER_MAX = 135;

constexpr float CAM_DEADZONE = 10.0f;
constexpr float CAM_MAX_DEV = 50.0f;
constexpr float BLINK_THRESH = 10.0f;

constexpr int ESC_NEUTRAL = 90;
constexpr int ESC_MIN_FWD = 95;
constexpr int ESC_MAX_FWD = 180;
constexpr int ESC_BRAKE = 20;

constexpr float MAX_SPEED_KMH = 15.0f;
constexpr float MAX_VALID_SPEED_KMH = 18.0f;

constexpr uint32_t PID_DT_US = 10000;  
constexpr uint32_t TELEM_DT_MS = 20;
constexpr uint32_t WDOG_TIMEOUT_MS = 500;
constexpr uint32_t FRAME_TIMEOUT_US = 8000;
constexpr uint32_t BRAKE_HOLD_MS = 200; // Đã bổ sung
constexpr float STOPPED_KMH = 0.3f;

// Lọc nhiễu siêu mượt cho camera
constexpr float ALPHA_STEER = 0.1f; 

constexpr uint32_t UART_BAUD = 230400; 
constexpr size_t RX_BUF_SIZE = 4096;
constexpr size_t TX_BUF_SIZE = 512;

constexpr uint8_t HDR_RX1 = 0xAB, HDR_RX2 = 0xCD; 
constexpr uint8_t HDR_TX1 = 0xDC, HDR_TX2 = 0xBA; 
constexpr uint8_t RX_LEN = 11;
constexpr uint8_t TX_LEN = 7;

constexpr uint32_t CPU_RX_BUDGET_US = 800; 
constexpr uint16_t MAX_RX_PER_LOOP = 2048;

// Chỉ mục vị trí dữ liệu trong gói tin (Đã bổ sung)
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

enum class RxState : uint8_t { WAIT_H1, WAIT_H2, READ_PAYLOAD };

// ============================================================================
// [4] BIẾN TOÀN CỤC (GLOBALS)
// ============================================================================
Servo servo_steer, motor_esc;

CarState car;
PidState pid_steer;

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

// ============================================================================
// [6] XỬ LÝ LÕI TÍNH TOÁN & GIAO TIẾP
// ============================================================================
void readUART() {
  uint32_t start_us = micros();

  if (rx_state != RxState::WAIT_H1 && (start_us - last_rx_us) > FRAME_TIMEOUT_US) {
    rx_state = RxState::WAIT_H1;
    rx_idx = 0;
  }

  uint16_t count = 0;

  while (Serial.available() > 0 && count < MAX_RX_PER_LOOP) {
    if ((micros() - start_us) >= CPU_RX_BUDGET_US) break;

    count++;
    uint8_t b = Serial.read();
    last_rx_us = micros();

    switch (rx_state) {
    case RxState::WAIT_H1:
      if (b == HDR_RX1) { rx_buf[0] = b; rx_idx = 1; rx_state = RxState::WAIT_H2; }
      break;
    case RxState::WAIT_H2:
      if (b == HDR_RX2) { rx_buf[1] = b; rx_idx = 2; rx_state = RxState::READ_PAYLOAD; }
      else if (b == HDR_RX1) { rx_buf[0] = b; rx_idx = 1; }
      else { rx_state = RxState::WAIT_H1; }
      break;
    case RxState::READ_PAYLOAD:
      if (rx_idx < RX_LEN) rx_buf[rx_idx++] = b;
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

int getBasePWM(float spd) {
  if (spd < 0.5f) return ESC_NEUTRAL;
  if (spd < ESC_LUT[2].spd) return ESC_MIN_FWD;

  for (size_t i = 0; i + 1 < LUT_SIZE; ++i) {
    if (spd >= ESC_LUT[i].spd && spd < ESC_LUT[i + 1].spd) {
      float dSpd = ESC_LUT[i + 1].spd - ESC_LUT[i].spd;
      if (dSpd <= 0.0f) return ESC_LUT[i].pwm;
      float ratio = (spd - ESC_LUT[i].spd) / dSpd;
      return roundf(ESC_LUT[i].pwm + ratio * (ESC_LUT[i + 1].pwm - ESC_LUT[i].pwm));
    }
  }
  return ESC_LUT[LUT_SIZE - 1].pwm;
}

void calcSteerPID(float dt) {
  car.smooth_dev = ALPHA_STEER * car.raw_dev + (1.0f - ALPHA_STEER) * car.smooth_dev;

  if (car.cur_spd < 10.0f) {
    pid_steer.kp = 0.3f; pid_steer.ki = 0.0f; pid_steer.kd = 1.5f;
  } else if (car.cur_spd < 25.0f) {
    pid_steer.kp = 0.25f; pid_steer.ki = 0.0f; pid_steer.kd = 1.8f;
  } else {
    pid_steer.kp = 0.2f; pid_steer.ki = 0.0f; pid_steer.kd = 2.0f;
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

  pid_steer.integral = 0; 
  float I = 0.0f;
  
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

  if ((int32_t)(now - next_pid_us) < 0) return;

  next_pid_us += PID_DT_US;
  if ((int32_t)(now - next_pid_us) >= 0) next_pid_us = now + PID_DT_US;

  constexpr float DT_SEC = PID_DT_US * 1e-6f;

  if (!car.emg_stop) {
    int target_pwm = getBasePWM(car.target_spd);
    car.esc_cmd = constrain(target_pwm, ESC_MIN_FWD, ESC_MAX_FWD);
    
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

  if (!car.braking) { car.braking = true; phase = 0; start_ms = now; }

  if (phase == 0) {
    if (last_esc != ESC_BRAKE) { motor_esc.write(ESC_BRAKE); last_esc = ESC_BRAKE; }
    if (now - start_ms >= BRAKE_HOLD_MS) { phase = 1; start_ms = now; }
  } else {
    if (last_esc != ESC_NEUTRAL) { motor_esc.write(ESC_NEUTRAL); last_esc = ESC_NEUTRAL; }
    car.esc_cmd = ESC_NEUTRAL;
    car.braking = false;
  }
}

void checkSafety() {
  if (millis() - last_packet_ms > WDOG_TIMEOUT_MS) {
    car.target_spd = 0; car.raw_dev = 0; car.emg_stop = true;
  }

  if (!car.emg_stop) {
    if (car.braking) {
      car.braking = false;
      if (last_esc != ESC_NEUTRAL) { motor_esc.write(ESC_NEUTRAL); last_esc = ESC_NEUTRAL; }
      car.esc_cmd = ESC_NEUTRAL; 
    }
    return;
  }

  if (car.braking) { processBrake(); return; }
  
  if (car.cur_spd > STOPPED_KMH) { processBrake(); return; }

  if (last_esc != ESC_NEUTRAL) { motor_esc.write(ESC_NEUTRAL); last_esc = ESC_NEUTRAL; }
  car.esc_cmd = ESC_NEUTRAL;
}

void sendTelemetry() {
  uint8_t tx_buf[TX_LEN] = {HDR_TX1, HDR_TX2, 0, 0, 0, 0, 0};
  memcpy(&tx_buf[2], &car.cur_spd, sizeof(float));
  tx_buf[6] = calcXor(tx_buf, 2, 5);
  if (Serial.availableForWrite() >= TX_LEN) Serial.write(tx_buf, TX_LEN);
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

  last_packet_ms = millis();
  next_pid_us = micros() + PID_DT_US;
  pid_timer_init = true;
}

void loop() {
  readUART();
  checkSafety();
  runPID(micros());

  static uint32_t last_tx = millis();
  if (millis() - last_tx >= TELEM_DT_MS) {
    sendTelemetry();
    last_tx = millis();
  }
}
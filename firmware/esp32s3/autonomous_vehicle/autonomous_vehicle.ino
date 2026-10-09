// ============================================================================
// BỘ ĐIỀU KHIỂN XE TỰ HÀNH - ESP32 (bản thường, ESP32-WROOM / DevKit V1)
// Phân hệ: Low-level Controller (Xử lý tín hiệu & Chấp hành)
// ----------------------------------------------------------------------------
// (Thư mục vẫn tên esp32s3/ cho khớp lịch sử repo; chip thật là ESP32 thường.)
//
// Giao tiếp với Mini PC: `Serial` = UART0 (GPIO1 TX / GPIO3 RX) qua chip
// USB-UART CP2102 trên board -> Mini PC thấy /dev/ttyUSB0, 230400 baud 8N1.
// ESP32 thường không có USB-OTG nên đây là đường serial duy nhất.
//
// Arduino IDE: Board "ESP32 Dev Module" (fqbn esp32:esp32:esp32),
//              Tools -> "Core Debug Level" = None (log ESP-IDF in ra UART0
//              sẽ chen vào khung nhị phân).
// Chân dùng (25, 26, 27, 32, 33, 4) đều là GPIO hợp lệ của ESP32 thường.
// ============================================================================

#include <Arduino.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================================
// [1] CẤU HÌNH CHÂN PHẦN CỨNG (PINS)
// ============================================================================
// !!! KIEM TRA DAY THAT: 2 sketch thu (motor_test, servo_test) dung ESC = GPIO 14,
// servo = GPIO 16, con firmware nay dung ESC = 33, servo = 32. Neu day tin hieu
// ESC dang cam o chan 14 thi doi PIN_ESC = 14 (va PIN_STEER = 16 neu servo o
// chan 16): ESP32 phat xung ra chan khong noi gi -> ESC dung yen du Mini PC da
// gui lenh chay. Telemetry v2 bao ve muc xung ESC dang phat de doi chieu.
constexpr uint8_t PIN_STEER = 32;
constexpr uint8_t PIN_ESC = 33;
constexpr uint8_t PIN_TURN_L = 25; 
constexpr uint8_t PIN_TURN_R = 26; 
constexpr uint8_t PIN_BRAKE  = 27; 
constexpr uint8_t PIN_HALL   = 4;  

// ============================================================================
// [2] THÔNG SỐ VẬT LÝ & ĐIỀU KHIỂN (PARAMETERS)
// ============================================================================
// ±35° (2026-10-09, trước ±30°): đánh lái rộng hơn ở cua. Servo kêu rè / gồng
// khi đánh hết lái = chạm cữ cơ khí -> giảm về 57/123.
constexpr int STEER_CENTER = 90;
constexpr int STEER_MIN = 55;
constexpr int STEER_MAX = 125;

// dev (px ảnh tham chiếu 640, ~1.5 mm/px ở 0.65 m). Deadzone nhỏ để lệch
// 1 cm đã bắt đầu kéo về giữa làn. Độ dốc giữ nguyên 30° / 41 px như bản ±30°
// (Mini PC cũ / mới đều ra đúng góc), chỉ bão hoà muộn hơn: 4 px -> 0°,
// ~52 px -> 35°.
constexpr float CAM_DEADZONE = 4.0f;
constexpr float CAM_MAX_DEV = CAM_DEADZONE + 41.0f * (STEER_MAX - STEER_CENTER) / 30.0f;
constexpr float BLINK_THRESH = 10.0f;

constexpr int ESC_NEUTRAL = 90;
constexpr int ESC_MIN_FWD = 95;
constexpr int ESC_MAX_FWD = 180;

// ---- Vung chet cua ESC ----
// Servo.write(goc) phat xung 1000 + goc*1000/180 us: 90 -> 1500 us (dung),
// 95 -> 1527 us, 97 -> 1538 us, 100 -> 1555 us. ESC xe RC co VUNG CHET quanh
// 1500 us (thuong +-30..50 us): xung 95-97 nam trong/sat vung chet -> ESC coi
// la dung, banh KHONG quay du Mini PC da gui lenh chay.
// SAN GA (muc nho nhat khi xe chay). Cach do: dat xe xuong san, chay sketch
// motor_test tang dan tu 1% (=95) cho toi khi banh vua quay, doi % ra goc
// (95 + (pct-1)*85/99). Do duoc: 101 (1561 us, ~5 km/h); duoi muc nay dong co
// BLDC quay cham va keu cot ket.
// XUNG ESC THAT (do 09/10): thu vien ESP32Servo mac dinh timer 10 bit ->
// xung chi co buoc 20000/1024 = 19.53 us. Servo.write(goc) ra
// 1000 + goc*1000/180 us roi CAT xuong boi so 19.53 us:
//   90 -> 1484.4 us (neutral)   95 -> 1523.4    100, 101 -> 1543.0 (CUNG xung)
//   102, 103, 104 -> 1562.5     105, 106 -> 1582.0
// => san "101" thuc ra = 100 (BLDC keu), moi lenh 5.0-5.8 km/h deu ra 1543,
// 5.9-7.9 km/h deu ra 1562.5: toc do chi nhay 2 bac, giam toc vao cua giat.
// Nay ESC dung timer 16 bit (0.3 us / buoc). Cac muc cu (neutral, phanh,
// de-pa, cac diem ESC_LUT) giu DUNG xung that cu (escLegacyUs) -> ESC thay
// y het ban cu; chi phan GIUA cac muc la moi (noi suy theo us).
constexpr int ESC_TIMER_BITS = 16;
constexpr float ESC_US_PER_TICK = 20000.0f / (1 << ESC_TIMER_BITS);
// SAN GA khi xe chay (us). Ban cu 1543 us: BLDC con keu (nhat la vao cua).
// 1562.5 us (toc do duong thang cu) khong keu. Van keu thi tang 2-3 us; muon
// cham hon thi giam, KHONG xuong duoi 1543. (Node con co speed_min_x10: san
// that = max(ESC_START_US, xung cua speed_min_x10), tang duoc khong can nap.)
constexpr float ESC_START_US = 1550.0f;
// BU TAI KHI DANH LAI: banh truoc be goc thi ma sat lon, cung muc xung dong co
// quay cham lai -> BLDC tut vong, keu dung luc vao cua. Cong them toi da
// ESC_STEER_BOOST_US khi servo het lai (ti le |goc lai| / 35 do). 0 = tat.
constexpr float ESC_STEER_BOOST_US = 3.0f;
// "De-pa": tu dung yen sang chay, ghi ESC_KICK_FWD trong ESC_KICK_MS de thang
// ma sat tinh, sau do moi ve muc ga cua toc do dat.
constexpr int ESC_KICK_FWD = 106;
constexpr uint32_t ESC_KICK_MS = 300;
constexpr int ESC_BRAKE = 20;

constexpr float MAX_SPEED_KMH = 15.0f;
constexpr float MAX_VALID_SPEED_KMH = 18.0f;

constexpr uint32_t PID_DT_US = 10000;  
constexpr uint32_t TELEM_DT_MS = 20;
constexpr uint32_t WDOG_TIMEOUT_MS = 500;
constexpr uint32_t FRAME_TIMEOUT_US = 8000;
constexpr uint32_t BRAKE_HOLD_MS = 200; // Đã bổ sung
constexpr float STOPPED_KMH = 0.3f;

// ---- Lái ----
// Mini PC đã lọc dev (chặn nhảy + EMA) nên ở đây chỉ lọc nhẹ để mượt servo.
// 0.5 ở 100 Hz: hằng số thời gian ~15 ms (bản cũ 0.25 ~ 35 ms).
constexpr float ALPHA_STEER = 0.5f;

// Góc lái = STEER_KP * map(dev) + D. map(): deadzone CAM_DEADZONE px, bão hoà
// ở CAM_MAX_DEV px -> 0..35°. KP = 1.0 dùng hết 35° cho cua gắt. Xe lắc qua
// lại trên đường thẳng thì giảm STEER_KP (0.8).
constexpr float STEER_KP = 1.0f;

// Khâu D tính trên tốc độ thay đổi của góc đã lọc (độ/giây), lọc thông thấp
// thêm 1 lần và giới hạn ±STEER_D_MAX độ để KHÔNG bị giật mỗi khi có frame mới.
constexpr float STEER_KD = 0.03f;       // giây
constexpr float STEER_D_ALPHA = 0.2f;   // lọc đạo hàm
constexpr float STEER_D_MAX = 6.0f;     // độ

// Tốc độ quay tối đa của lệnh servo (độ/giây): chặn giật cơ khí
constexpr float STEER_RATE_DEG_S = 500.0f;

constexpr uint32_t UART_BAUD = 230400; 
constexpr size_t RX_BUF_SIZE = 4096;
constexpr size_t TX_BUF_SIZE = 512;

constexpr uint8_t HDR_RX1 = 0xAB, HDR_RX2 = 0xCD; 
constexpr uint8_t HDR_TX1 = 0xDC, HDR_TX2 = 0xBA; // telemetry cu, 7 byte (khong con gui)
// Telemetry v2, 10 byte, ~50 Hz:
//   DC BB | float32 cur_spd | ESC_DEG | STEER_DEG | FLAGS | XOR(byte 2..8)
//   ESC_DEG  : goc Servo dang ghi cho ESC (90 = neutral, 95..180 = tien)
//   STEER_DEG: goc servo lai dang ghi
//   FLAGS    : bit0 = dang EMG, bit1 = watchdog (mat lenh > 500 ms),
//              bit2 = dang phanh, bit3 = da nhan it nhat 1 goi hop le
constexpr uint8_t HDR_TX2_V2 = 0xBB;
constexpr uint8_t TX_LEN_V2 = 10;
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

struct SteerState {
  float prev_angle = 0.0f;   // góc mục tiêu (lệch khỏi tâm) chu kỳ trước
  float d_filt = 0.0f;       // đạo hàm đã lọc (độ/giây)
  float out = STEER_CENTER;  // lệnh servo sau giới hạn tốc độ quay
};

enum class RxState : uint8_t { WAIT_H1, WAIT_H2, READ_PAYLOAD };

// ============================================================================
// [4] BIẾN TOÀN CỤC (GLOBALS)
// ============================================================================
Servo servo_steer, motor_esc;

CarState car;
SteerState steer;

RxState rx_state = RxState::WAIT_H1;
uint8_t rx_buf[RX_LEN] = {};
uint8_t rx_idx = 0;
uint32_t last_rx_us = 0;
uint32_t last_packet_ms = 0;
bool got_packet = false;   // da nhan it nhat 1 goi hop le tu Mini PC

uint32_t next_pid_us = 0;
bool pid_timer_init = false;

int last_esc = ESC_NEUTRAL;      // goc tuong duong xung that (telemetry)
float last_esc_us = 0.0f;        // xung that dang phat (us), 0 = chua ghi
int last_esc_ticks = -1;
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
          got_packet = true;
        }
        rx_state = RxState::WAIT_H1;
        rx_idx = 0;
      }
      break;
    }
  }
}

// Xung THAT (us) ban cu phat ra khi goi Servo.write(deg) voi timer 10 bit:
// map(deg, 0, 180, 1000, 2000) roi cat xuong boi so 20000/1024 us
float escLegacyUs(int deg) {
  constexpr float tick10 = 20000.0f / 1024.0f;
  const long us = 1000L + (long)deg * 1000L / 180L;
  return (float)(long)(us / tick10) * tick10;
}

// Ghi xung ESC (us, cho phep le) neu khac xung dang phat. last_esc = goc
// tuong duong cua xung THAT (telemetry: 1543 us -> 98, 1552 -> 99, 1562.5 -> 101)
void escWriteUs(float us) {
  const int ticks = (int)lroundf(us / ESC_US_PER_TICK);
  if (ticks == last_esc_ticks) return;
  motor_esc.writeTicks(ticks);
  last_esc_ticks = ticks;
  last_esc_us = ticks * ESC_US_PER_TICK;
  last_esc = (int)lroundf((last_esc_us - 1000.0f) * 0.18f);
}

// Toc do (km/h) -> xung (us): noi suy ESC_LUT theo xung THAT cua tung diem
float getBaseUs(float spd) {
  if (spd < 0.5f) return escLegacyUs(ESC_NEUTRAL);
  if (spd < ESC_LUT[2].spd) return escLegacyUs(ESC_MIN_FWD);

  for (size_t i = 0; i + 1 < LUT_SIZE; ++i) {
    if (spd >= ESC_LUT[i].spd && spd < ESC_LUT[i + 1].spd) {
      const float u0 = escLegacyUs(ESC_LUT[i].pwm);
      const float u1 = escLegacyUs(ESC_LUT[i + 1].pwm);
      const float dSpd = ESC_LUT[i + 1].spd - ESC_LUT[i].spd;
      if (dSpd <= 0.0f) return u0;
      return u0 + (spd - ESC_LUT[i].spd) / dSpd * (u1 - u0);
    }
  }
  return escLegacyUs(ESC_LUT[LUT_SIZE - 1].pwm);
}

// Tính góc servo từ dev (px ảnh tham chiếu 640, dev < 0 = tâm làn lệch trái).
//   1. Lọc EMA dev.
//   2. map(|dev|): deadzone CAM_DEADZONE -> 0°, CAM_MAX_DEV -> 35°.
//   3. P = STEER_KP * góc; D = STEER_KD * d(góc)/dt (đã lọc, có giới hạn).
//   4. Giới hạn tốc độ quay servo STEER_RATE_DEG_S.
// Bản cũ lấy D = 1.5 * Δgóc / 0.01 s: mỗi frame camera mới làm D vọt lên
// hàng chục độ -> servo giật hết lái rồi mới về, xe lắc.
void calcSteer(float dt) {
  car.smooth_dev = ALPHA_STEER * car.raw_dev + (1.0f - ALPHA_STEER) * car.smooth_dev;

  float angle = 0.0f;  // độ lệch khỏi tâm, + = lái trái (servo > 90)
  const float abs_dev = fabsf(car.smooth_dev);
  if (abs_dev > CAM_DEADZONE) {
    const float span = CAM_MAX_DEV - CAM_DEADZONE;
    const float off = mapF(constrain(abs_dev - CAM_DEADZONE, 0.0f, span), 0.0f, span,
                           0.0f, (float)(STEER_MAX - STEER_CENTER));
    angle = (car.smooth_dev < 0.0f) ? off : -off;
  }

  const float d_raw = (angle - steer.prev_angle) / dt;
  steer.prev_angle = angle;
  steer.d_filt += STEER_D_ALPHA * (d_raw - steer.d_filt);
  const float D = constrain(STEER_KD * steer.d_filt, -STEER_D_MAX, STEER_D_MAX);

  const float target = constrain(STEER_CENTER + STEER_KP * angle + D,
                                 (float)STEER_MIN, (float)STEER_MAX);

  const float max_step = STEER_RATE_DEG_S * dt;
  steer.out += constrain(target - steer.out, -max_step, max_step);
  car.steer_cmd = (int)roundf(steer.out);
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
    static uint32_t kick_until_ms = 0;
    const uint32_t now_ms = millis();

    const float neutral_us = escLegacyUs(ESC_NEUTRAL);
    float us = neutral_us;
    if (car.target_spd >= 0.5f) {
      // Dang chay: khong de ga xuong duoi san (vung chet ESC / BLDC keu)
      us = max(getBaseUs(car.target_spd), ESC_START_US);
      // Bu tai theo goc lai dang ra lenh (chu ky truoc)
      const float steer_frac = fabsf(steer.out - STEER_CENTER) / (float)(STEER_MAX - STEER_CENTER);
      us += ESC_STEER_BOOST_US * constrain(steer_frac, 0.0f, 1.0f);
      // Vua tu dung yen (ESC dang neutral) chuyen sang chay -> de-pa
      if (last_esc_us <= neutral_us + 1.0f) kick_until_ms = now_ms + ESC_KICK_MS;
      if ((int32_t)(kick_until_ms - now_ms) > 0) us = max(us, escLegacyUs(ESC_KICK_FWD));
      us = constrain(us, escLegacyUs(ESC_MIN_FWD), escLegacyUs(ESC_MAX_FWD));
    }
    // Lenh toc do ~0 nhung khong EMG: dung (truoc day bi kep len 95 -> bo)
    escWriteUs(us);
    car.esc_cmd = last_esc;
  }

  calcSteer(DT_SEC);
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
    escWriteUs(escLegacyUs(ESC_BRAKE));
    if (now - start_ms >= BRAKE_HOLD_MS) { phase = 1; start_ms = now; }
  } else {
    escWriteUs(escLegacyUs(ESC_NEUTRAL));
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
      escWriteUs(escLegacyUs(ESC_NEUTRAL));
      car.esc_cmd = ESC_NEUTRAL; 
    }
    return;
  }

  if (car.braking) { processBrake(); return; }
  
  if (car.cur_spd > STOPPED_KMH) { processBrake(); return; }

  escWriteUs(escLegacyUs(ESC_NEUTRAL));
  car.esc_cmd = ESC_NEUTRAL;
}

// Gui trang thai len Mini PC: van toc + muc xung ESC/servo THAT SU dang phat +
// co trang thai. Nho vay GUI biet lenh chay co toi duoc ESC hay khong.
void sendTelemetry() {
  uint8_t tx_buf[TX_LEN_V2] = {HDR_TX1, HDR_TX2_V2};
  memcpy(&tx_buf[2], &car.cur_spd, sizeof(float));
  tx_buf[6] = (uint8_t)constrain(last_esc, 0, 180);
  tx_buf[7] = (uint8_t)constrain(last_steer, 0, 180);
  uint8_t flags = 0;
  if (car.emg_stop) flags |= 0x01;
  if (millis() - last_packet_ms > WDOG_TIMEOUT_MS) flags |= 0x02;
  if (car.braking) flags |= 0x04;
  if (got_packet) flags |= 0x08;
  tx_buf[8] = flags;
  tx_buf[9] = calcXor(tx_buf, 2, 8);
  if (Serial.availableForWrite() >= TX_LEN_V2) Serial.write(tx_buf, TX_LEN_V2);
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
  // attach() luon dat lai timer 10 bit -> doi sang 16 bit SAU attach
  motor_esc.setTimerWidth(ESC_TIMER_BITS);
  escWriteUs(escLegacyUs(ESC_NEUTRAL));

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
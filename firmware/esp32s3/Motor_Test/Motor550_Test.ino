#include <ESP32Servo.h>

// GPIO 18 là PIN_TURN_R của Autonomous_Vehicle — chấn cùng đó sẽ quay xi-nhan
// mỗi lần đổi hướng. Xe chính dùng PIN_ESC = 14, dùng lại 14 cho test.
#define ESC_PIN 14

Servo esc;

// Xe chính điều khiển ESC bằng góc SERVO (attach(pin, 1000, 2000)) chứ không
// phải micro-giây. Giữ đúng cùng dải góc với Autonomous_Vehicle.ino
// (motor_esc.attach(PIN_ESC, 1000, 2000), ESC_NEUTRAL=90, MIN_FWD=95,
// MAX_FWD=180) để kết quả test khớp xe thật.
const int FWD_MIN = 95;  // = ESC_MIN_FWD
const int FWD_MAX = 180; // = ESC_MAX_FWD
const int STOP_DEG = 90; // = ESC_NEUTRAL

// Ngưỡng lùi/phanh (ESC_BRAKE = 20 độ trong sketch chính)
const int REV_MIN = 40;
const int REV_MAX = 0;

void setMotorSpeed(int percent) {
  percent = constrain(percent, -100, 100);

  int dutyDeg = STOP_DEG;

  if (percent > 0) {
    // 1% -> ESC_MIN_FWD, 100% -> ESC_MAX_FWD (độ, khớp sketch chính)
    dutyDeg = map(percent, 1, 100, FWD_MIN, FWD_MAX);
  } else if (percent < 0) {
    dutyDeg = map(percent, -1, -100, REV_MIN, REV_MAX);
  }

  esc.write(dutyDeg);
  Serial.printf("Toc do: %3d%% -> ESC goc: %d do\n", percent, dutyDeg);
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);

  // Arming — phải đặt NEUTRAL (90 độ), KHÔNG dùng writeMicroseconds(1500)
  // vì ESC ở đây điều khiển bằng góc, 1500us ≈ 150 độ = chạy hết tốc tiến.
  esc.write(STOP_DEG);
  delay(3000);
  Serial.println("ESC da san sang. Nhap toc do tu -100 den 100:");
}

void loop() {
  if (Serial.available() > 0) {
    int spd = Serial.parseInt();
    // Đọc bỏ ký tự thừa như \n, \r
    while (Serial.available())
      Serial.read();

    setMotorSpeed(spd);
  }
}
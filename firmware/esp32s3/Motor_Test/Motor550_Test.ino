#include <ESP32Servo.h>

#define ESC_PIN 18

Servo esc;

// Các ngưỡng xung thực tế đã đo đạc
const int FWD_MIN = 1600; // Ngưỡng bắt đầu quay tiến
const int FWD_MAX = 2000; // Max công suất tiến
const int REV_MIN = 1400; // Ngưỡng bắt đầu quay lùi
const int REV_MAX = 1000; // Max công suất lùi
const int STOP_US = 1500; // Dừng hoàn toàn

void setMotorSpeed(int percent) {
  percent = constrain(percent, -100, 100);

  int pulseUs = STOP_US;

  if (percent > 0) {
    // Ánh xạ dải 1% -> 100% thẳng vào 1600us -> 2000us
    pulseUs = map(percent, 1, 100, FWD_MIN, FWD_MAX);
  } else if (percent < 0) {
    // Ánh xạ dải -1% -> -100% thẳng vào 1400us -> 1000us
    pulseUs = map(percent, -1, -100, REV_MIN, REV_MAX);
  } else {
    pulseUs = STOP_US;
  }

  esc.writeMicroseconds(pulseUs);
  Serial.printf("Toc do: %3d%% -> Xung: %d us\n", percent, pulseUs);
}

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  esc.setPeriodHertz(50);
  esc.attach(ESC_PIN, 1000, 2000);

  // Arming
  esc.writeMicroseconds(STOP_US);
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
#include <ESP32Servo.h>

Servo servo;
// GPIO 18 là PIN_TURN_R của Autonomous_Vehicle — dùng lại sẽ lái xi-nhan
// ngẫu nhiên khi chạy test servo. Chọn GPIO 16 (chân tự do, ngoài USB 19/20).
#define SERVO_PIN 16

int currentAngle = 90;

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo.setPeriodHertz(50);
  servo.attach(SERVO_PIN, 500, 2400);

  // Đưa về góc 90 mặc định
  servo.write(currentAngle);

  Serial.println("--- TOOL CAN CHINH THUOC LAI ---");
  Serial.println("Go 'a' : Nhich 1 do sang TRAI");
  Serial.println("Go 'd' : Nhich 1 do sang PHAI");
  Serial.println("Go 'q' : Nhich 5 do sang TRAI (nhanh)");
  Serial.println("Go 'e' : Nhich 5 do sang PHAI (nhanh)");
  Serial.println("Go 'c' : Ve tam 90 do");
  Serial.println("--------------------------------");
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();

    bool changed = false;

    if (cmd == 'a') { currentAngle--; changed = true; }
    else if (cmd == 'd') { currentAngle++; changed = true; }
    else if (cmd == 'q') { currentAngle -= 5; changed = true; }
    else if (cmd == 'e') { currentAngle += 5; changed = true; }
    else if (cmd == 'c') { currentAngle = 90; changed = true; }

    if (changed) {
      currentAngle = constrain(currentAngle, 0, 180);
      servo.write(currentAngle);
      Serial.printf("Goc hien tai: %d do\n", currentAngle);
    }
  }
}
#include <Arduino.h>
#define ENCODER_1A 36 // Pin for Encoder A
#define ENCODER_1B 39 // Pin for Encoder B


volatile int encoder1_value = 0; // Global variable for storing the encoder position

void encoder1_isr() { // Mode: change
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_1A);
  int B = digitalRead(ENCODER_1B);

  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == HIGH)) {
    encoder1_value++; //cw
  } else {
    encoder1_value--; //ccw
  }
}

void setup() 
{
  Serial.begin(115200); // Initialize serial communication
  pinMode(ENCODER_1A, INPUT_PULLUP);
  pinMode(ENCODER_1B, INPUT_PULLUP);

  pinMode(ENCODER_2A, INPUT_PULLUP);
  pinMode(ENCODER_2B, INPUT_PULLUP);


  // Attaching the ISR to encoder 1A
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), encoder1_isr, CHANGE);

}

void loop() 
{
  // tinh goc quay: angle = (pulsesCurrent) * 360/(RPM: so xung tren 1 vong)
  //Tinh goc quay khi quay hon 1 vong
  // int pulsesCurrenRound = encoder1_value % 666;

  Serial.print("Encoder 1 value: " + String(encoder1_value) + "\tSo vong quay: " + String(encoder1_value/666) + "\tGoc quay: " + String(((encoder1_value % 666) * 360) / 666) + "\t&&||\t");
  Serial.println("Encoder 2 value: " + String(encoder2_value) + "\tSo vong quay: " + String(encoder2_value/333) + "\tGoc quay: " + String(((encoder2_value % 333) * 360) / 333) );
}
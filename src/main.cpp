#include "globals.hpp"

// Create an instance of EV
EV ev(MOTOR_A1, MOTOR_A2, MOTOR_ENA, 
       MOTOR_B1, MOTOR_B2, MOTOR_ENB, 
       ENCODER_A1, ENCODER_A2, ENCODER_B1, ENCODER_B2, 
       START_BUTTON
      );

void getSensors() {
  ev.updateEncoder(ev.sensor.leftEncoderCount, ev.pinSA1, ev.pinSA2);
  ev.updateEncoder(ev.sensor.rightEncoderCount, ev.pinSB1, ev.pinSB2);

}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Wake up MPU6050 by clearing the sleep bit in the power management register (0x6B)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // Give sensor time to stabilize
  delay(100);

  ev.initialize();

  // Calibrate the gyro (make sure the sensor is still)
  Serial.println("Calibrating Gyro... Do not move the MPU6050!");
  calibrateGyroZ();
  Serial.println("Calibration complete.");

  lastTime = millis();

  attachInterrupt(digitalPinToInterrupt(ev.pinSA1), getSensors, CHANGE);
}

// testing values
// double distance = 75; // make sure this is in cm (29 in)
// double distance = 15; // make sure this is in cm (6 in)
// double distance = 61; // make sure this is in cm (24 in)
// double distance = 122; // make sure this is in cm (48 in)
// double distance = 200; // make sure this is in cm (84 in)


double distance = 700; // make sure this is in cm

int loopCount = 1;
void loop() {
  if (digitalRead(ev.pinButton) == HIGH) {
    delay(100);
    if (loopCount == 1) { // make sure we only go once
      ev.PIDLoop(distance);
      loopCount++;
    }
  }
  ev.brake();
}




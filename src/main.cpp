#include "globals.hpp"
#include "EV.hpp"

// Create an instance of EV
EV ev(MOTOR_A1, MOTOR_A2, MOTOR_ENA, 
       MOTOR_B1, MOTOR_B2, MOTOR_ENB, 
       ENCODER_A1, ENCODER_A2, ENCODER_B1, ENCODER_B2, 
       START_BUTTON
      );

void getSensors() {
  ev.updateEncoder(ev.sensor.leftEncoderCount, ev.pinSA1, ev.pinSA2, true);
  ev.updateEncoder(ev.sensor.rightEncoderCount, ev.pinSB1, ev.pinSB2, false);

}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  ev.initialize();

  // Initialize and zero the BNO055 heading
  Serial.println("Starting BNO055...");
  if (!setupIMU()) {
    while (true) {
      Serial.println("IMU not detected. Check wiring.");
      delay(1000);
    }
  }
  delay(50);
  Serial.println("Hold still while heading zeroes...");
  calibrateHeadingZero();
  Serial.println("Heading zeroed.");

  attachInterrupt(digitalPinToInterrupt(ev.pinSA1), getSensors, CHANGE);
}

// testing values
// double distance = 75; // make sure this is in cm (29 in)
// double distance = 15; // make sure this is in cm (6 in)
// double distance = 61; // make sure this is in cm (24 in)
// double distance = 122; // make sure this is in cm (48 in)
// double distance = 200; // make sure this is in cm (84 in)


double distance = 750; // make sure this is in cm
double travelTimeSec = 20; // target travel time in seconds

int loopCount = 1;
void loop() {
  // if (digitalRead(ev.pinButton) == HIGH) {
  //   delay(100);
  //   if (loopCount == 1) { // make sure we only go once
  //     ev.PIDLoop(distance);
  //     loopCount++;
  //   }
  // }
  // ev.brake();
  ev.PIDLoop(distance, travelTimeSec);
  while (true) {
    delay(1000);
  }

  // double heading = readHeadingDeg();
  // Serial.print("Heading: ");
  // Serial.println(heading);
}



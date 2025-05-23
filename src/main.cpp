#include <Arduino.h>
#include <Encoder.h>

// === Pin Definitions ===
const int MOTOR_IN1 = 9;    // Direction pin 1
const int MOTOR_IN2 = 10;    // Direction pin 2
const int MOTOR_ENA = 11;    // PWM speed pin (ENA on L298N)
const int ENC_A = 3;
const int ENC_B = 2;
const int BUTTON_PIN = 4;

// === Motor Control (L298N) ===
void setMotorSpeed(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, pwm);
  } else if (pwm < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, -pwm);
  } else {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, 0);
  }
}

// === Encoder Setup ===
Encoder motorEncoder(ENC_A, ENC_B);

// === Movement Parameters ===
volatile bool startMovement = false;
long targetDistance_cm = 200;
float encoderTicksPerCm = 0.123; // ← Adjust based on your setup

// === PID Parameters ===
float Kp = 2.0, Ki = 0.5, Kd = 0.1;
float pidSum = 0;
float lastError = 0;
unsigned long lastTime = 0;

// === Button ISR ===
void onButtonPress() {
  startMovement = true;
}

void setup() {
  Serial.begin(9600);

  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);

  // pinMode(BUTTON_PIN, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress, FALLING);
  pinMode(BUTTON_PIN, INPUT);

  motorEncoder.write(0);
}

void loop() {
  if (digitalRead(BUTTON_PIN) == LOW) {
    startMovement = false;

    long targetTicks = targetDistance_cm * encoderTicksPerCm;
    long startTicks = motorEncoder.read();
    double pidSum = 0;
    double lastError = 0;
    double lastTime = millis();

    while (true) {
      long currentTicks = motorEncoder.read() - startTicks;
      long error = targetTicks - currentTicks;

      if (abs(error) < 5) { // Stop condition
        setMotorSpeed(0);
        break;
      }

      // === PID Controller ===
      unsigned long now = millis();
      float deltaT = (now - lastTime) / 1000.0;
      float dError = (error - lastError) / deltaT;
      pidSum += error * deltaT;

      float output = Kp * error + Ki * pidSum + Kd * dError;
      setMotorSpeed(output);

      lastError = error;
      lastTime = now;
      delay(10); // 100 Hz loop
    }

    Serial.println("Target distance reached.");
  }
  Serial.println("Start button not pressed.");
}

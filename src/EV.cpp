#include "EV.hpp"

EV* EV::instance = nullptr; // Define static instance

// distOneTick is calculated by circumference / ppr ((4.7 * pi) / 120) / 2
// const double EV::distOneTick = 0.0615;
// const double EV::distOneTick = 0.03075;
const double EV::distOneTick = 0.123;
// const double EV::distOneTick = 0.246;





EV::EV(int p1, int p2, int pA, int p3, int p4, int pB, 
    int pSA1, int pSA2, int pSB1, int pSB2, 
    int pinButn)

    : pinA1(p1), pinA2(p2), pinENA(pA), pinB1(p3), pinB2(p4), pinENB(pB),
      pinSA1(pSA1), pinSA2(pSA2), pinSB1(pSB1), pinSB2(pSB2), pinButton(pinButn),
      pos(0, 0), sensor(), linPID(5.5, 1.5, 2) {
    
    instance = this; // Assign instance for interrupt handling
}

void EV::initialize() {
    pinMode(pinA1, OUTPUT); pinMode(pinA2, OUTPUT); pinMode(pinENA, OUTPUT);
    pinMode(pinB1, OUTPUT); pinMode(pinB2, OUTPUT); pinMode(pinENB, OUTPUT);
    pinMode(pinSA1, INPUT_PULLUP); pinMode(pinSA2, INPUT_PULLUP);
    pinMode(pinSB1, INPUT_PULLUP); pinMode(pinSB2, INPUT_PULLUP);
    pinMode(pinButton, INPUT);

    // attachInterrupt(digitalPinToInterrupt(pinSA1), EV::getSensorsOnInterupt, CHANGE);

    delay(100);
}


//Driver methods
void EV::forward(uint8_t speed1, uint8_t speed2) {
    analogWrite(pinENA, speed1); digitalWrite(pinA1, HIGH); digitalWrite(pinA2, LOW);
    analogWrite(pinENB, speed2); digitalWrite(pinB1, HIGH); digitalWrite(pinB2, LOW);
}

void EV::backward(uint8_t speed) {
    analogWrite(pinENA, speed); digitalWrite(pinA1, LOW); digitalWrite(pinA2, HIGH);
    analogWrite(pinENB, speed); digitalWrite(pinB1, LOW); digitalWrite(pinB2, HIGH);
}

void EV::left(uint8_t speed) {
    analogWrite(pinENA, speed); digitalWrite(pinA1, LOW); digitalWrite(pinA2, HIGH);
    analogWrite(pinENB, speed); digitalWrite(pinB1, HIGH); digitalWrite(pinB2, LOW);
}

void EV::right(uint8_t speed) {
    analogWrite(pinENA, speed); digitalWrite(pinA1, HIGH); digitalWrite(pinA2, LOW);
    analogWrite(pinENB, speed); digitalWrite(pinB1, LOW); digitalWrite(pinB2, HIGH);
}

void EV::brake() {
    analogWrite(pinENA, 0); digitalWrite(pinA1, LOW); digitalWrite(pinA2, LOW);
    analogWrite(pinENB, 0); digitalWrite(pinB1, LOW); digitalWrite(pinB2, LOW);
}


// Sesnsor methods
void EV::tarePosition() {
    sensor.leftEncoderCount = 0;
    sensor.rightEncoderCount = 0;
}

void EV::updateEncoder(volatile int &encoderCount, int pinA, int pinB) {
    encoderCount += (digitalRead(pinA) == digitalRead(pinB)) ? -1 : 1; // 600 rpm
    // encoderCount += (digitalRead(pinA) == digitalRead(pinB)) ? -1 : 1; // 300 rpm

}

void EV::getSensorsOnInterupt() {
    if (instance) {
        instance->updateEncoder(instance->sensor.leftEncoderCount, instance->pinSA1, instance->pinSA2);
        instance->updateEncoder(instance->sensor.rightEncoderCount, instance->pinSB1, instance->pinSB2);
        return;
    }
    Serial.println("No instance found");
}

double EV::getDistance() {
    return ((sensor.leftEncoderCount + sensor.rightEncoderCount) / 2) * distOneTick;
    // return sensor.leftEncoderCount * distOneTick;
}

void EV::compLoop(double goal) {

    // goal -= 4.8387;
    
    tarePosition();


    while (true) {
        // updateEncoder(sensor.leftEncoderCount, pinSA1, pinSA2);
        double dist = getDistance();

        Serial.print("Total distance: ");
        Serial.println(dist);

        double error = goal - dist;

        if (error <= 2) {
            break;
        }

        forward(255, 255);
        delay(10);
        Serial.print("left ticks: ");
        Serial.print(sensor.leftEncoderCount);
        Serial.print(", right ticks: ");
        Serial.println(sensor.rightEncoderCount);
    }
    brake();
}


// PID methods
void EV::PIDLoop(double goal) {

    tarePosition();

    linPID.error = 0;
    linPID.derivative = 0;
    linPID.integral = 0;

    double correctKp = 285; // aims to fix drift 185, 285
    double correctKd = 2.5; // aims to fix drift 1.75, 2.5

    double correctPrevError = 0;
    double correctError = 0;
    double correctDerivative = 0;

    double prevTime = millis();
    double totalTime = 0;


    while (true) {
        double dist = getDistance(); // get's our average distance travelled

        linPID.error = goal - dist;
        Serial.print("error: ");
        Serial.println(linPID.error);

        // Get time delta in seconds
        double currentTime = millis();
        double dt = (currentTime - prevTime) / 1000.0;
        totalTime += dt;

        // account for detla time in these calculations
        linPID.derivative = (linPID.error - linPID.prevError) / dt;

        linPID.integral += linPID.error * dt;

        // clamp integral term to prevent windup
        linPID.integral = constrain(linPID.integral, -linPID.high, linPID.high);


        int32_t power = (linPID.kP * linPID.error) + 
                        (linPID.kI * linPID.integral) + 
                        (linPID.kD * linPID.derivative);
        
          // Read raw z-axis gyroscope data and subtract calibration offset
        int16_t gz = readGyroZ();
        float gyroZ = (gz - gyroOffsetZ) / GYRO_SCALE; // in °/s

        yaw += gyroZ * dt;  // yaw in degrees

        Serial.print("Yaw: ");
        Serial.println(yaw);

        correctError = yaw;
        correctDerivative = (correctError - correctPrevError) / dt;

        double correction = (correctKp * correctError) + 
                            (correctKd * correctDerivative);


        double leftPower = power + correction; // -s
        double rightPower = power - correction; // +

        leftPower = constrain(leftPower, linPID.low, linPID.high);
        rightPower = constrain(rightPower, linPID.low, linPID.high);
    
        
        // Check for timeout (using currentTime if you want an absolute time based on millis)
        if ((totalTime) >= linPID.timeOut) {
            Serial.println("broke bc time limit met");
            break;
        } 

        // Check if error is within an acceptable range
        if (abs(linPID.error) < 10.0) {
            Serial.println("broke bc target met");
            break;
        }

        // Move motors; if target is negative, error will be negative and so will the power
        if (power >= 0) forward(leftPower, rightPower);
        else backward(abs(power));
        

        // Update previous error and time for next loop
        linPID.prevError = linPID.error;
        correctPrevError = correctError;
        prevTime = currentTime;

        delay(10);
    }
    brake();
}

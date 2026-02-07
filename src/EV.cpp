#include "EV.hpp"
#include "globals.hpp"

EV* EV::instance = nullptr; // Define static instance

// all outdated and for old versions
// distOneTick is calculated by circumference / ppr ((4.7 * pi) / 120) / 2
// const double EV::distOneTick = 0.0615;
// const double EV::distOneTick = 0.03075;
// const double EV::distOneTick = 0.123;  // outdated
// const double EV::distOneTick = 0.246;


/*
gear_ratio = 10
12 PPR × 4 (quadrature) = 48 counts per motor revolution
CPR_wheel = 48 × 20 = 960 counts per wheel revolution
Circumference = π × D
              = π × 54
              ≈ 169.65 mm
distOneTick = 169.65 mm / 960
            ≈ 0.1766 mm per tick
const double EV::distOneTick = 0.353; // mm per encoder count``

*/
// I'M ONLY USING ONE INTERRUPT PIN SO WE ONLY ACCOUNT FOR HALF THE TICKS PER REVOLUTION, NOT THE FULL QUADRATURE COUNT
// distOneTick in centimeters for consistency with distance goals
// const double EV::distOneTick = 0.0766;
// const double EV::distOneTick = 0.1002;
// const double EV::distOneTick = 0.0586;
// const double EV::distOneTick = 0.0646;
const double EV::distOneTick = 0.0676;



/*
Goal | Actual
48.     39
24.     17
6.     2.5
29.    22.5
84.    64
*/







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
    // Left motor wiring flipped, so invert drive signals for left side
    analogWrite(pinENA, speed1); digitalWrite(pinA1, LOW); digitalWrite(pinA2, HIGH);
    analogWrite(pinENB, speed2); digitalWrite(pinB1, HIGH); digitalWrite(pinB2, LOW);
}

void EV::backward(uint8_t speed) {
    analogWrite(pinENA, speed); digitalWrite(pinA1, HIGH); digitalWrite(pinA2, LOW);
    analogWrite(pinENB, speed); digitalWrite(pinB1, LOW); digitalWrite(pinB2, HIGH);
}

void EV::left(uint8_t speed) {
    // Turn left: left wheel backward (relative to robot), right forward
    analogWrite(pinENA, speed); digitalWrite(pinA1, HIGH); digitalWrite(pinA2, LOW);
    analogWrite(pinENB, speed); digitalWrite(pinB1, HIGH); digitalWrite(pinB2, LOW);
}

void EV::right(uint8_t speed) {
    // Turn right: left wheel forward, right backward
    analogWrite(pinENA, speed); digitalWrite(pinA1, LOW); digitalWrite(pinA2, HIGH);
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

void EV::updateEncoder(volatile int &encoderCount, int pinA, int pinB, bool inverted) {
    int step = (digitalRead(pinA) == digitalRead(pinB)) ? -1 : 1;
    if (inverted) step = -step;
    encoderCount += step; // 600 rpm
}

void EV::getSensorsOnInterupt() {
    if (instance) {
        instance->updateEncoder(instance->sensor.leftEncoderCount, instance->pinSA1, instance->pinSA2, true);
        instance->updateEncoder(instance->sensor.rightEncoderCount, instance->pinSB1, instance->pinSB2, false);
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
void EV::PIDLoop(double goal, double travelTimeSec) {

    tarePosition();

    linPID.error = 0;
    linPID.derivative = 0;
    linPID.integral = 0;

    double headingKp = 4.0;   // steering proportional gain
    double headingKd = 0.2;   // steering derivative gain
    double headingPrevError = 0.0;

    const uint8_t minPWM = 40;      // motors stall below this
    const uint8_t slowPWM = 50;     // gentle approach speed
    const double slowDist = 20.0;   // cm where we creep in
    const double scheduleKp = 4.0;  // adjust pace vs. time schedule

    double prevTime = millis();
    double startTime = prevTime;


    while (true) {
        double dist = -getDistance(); // average distance travelled (negated for wiring orientation)
        Serial.print("dist (cm): ");
        Serial.println(dist);

        linPID.error = goal - dist;
        Serial.print("error: ");
        Serial.println(linPID.error);

        // Get time delta in seconds
        double currentTime = millis();
        double dt = (currentTime - prevTime) / 1000.0;
        if (dt <= 0.0001) {
            dt = 0.0001; // prevent divide-by-zero during fast loops
        }

        // account for detla time in these calculations
        linPID.derivative = (linPID.error - linPID.prevError) / dt;

        linPID.integral += linPID.error * dt;

        // clamp integral term to prevent windup
        linPID.integral = constrain(linPID.integral, -linPID.high, linPID.high);

        int32_t power = (linPID.kP * linPID.error) + 
                (linPID.kI * linPID.integral) + 
                (linPID.kD * linPID.derivative);

        // Time-based pacing: if behind schedule, push harder; if ahead, ease off
        double elapsed = (currentTime - startTime) / 1000.0;
        double expectedDist = goal * min(elapsed / travelTimeSec, 1.0);
        double scheduleError = dist - expectedDist; // positive = ahead
        double scheduleAdjust = -scheduleKp * scheduleError; // slow when ahead, speed when behind

        // Heading hold using differential drive
        double headingError = readHeadingDeg();
        double headingDerivative = (headingError - headingPrevError) / dt;
        double headingCorrection = (headingKp * headingError) + (headingKd * headingDerivative);

        double rawBase = fabs(power + scheduleAdjust);

        // Final approach: creep in; allow schedule adjust to slow us further if ahead
        if (fabs(linPID.error) <= slowDist) {
            rawBase = min(rawBase, static_cast<double>(slowPWM));
        }

        int32_t basePower = constrain(static_cast<int32_t>(rawBase), minPWM, 255);
        int32_t leftPower = constrain(basePower - headingCorrection, minPWM, 255);
        int32_t rightPower = constrain(basePower + headingCorrection, minPWM, 255);

        forward(static_cast<uint8_t>(leftPower), static_cast<uint8_t>(rightPower));
    
        // Check if error is within an acceptable range
        if (abs(linPID.error) < 5.0) {
            Serial.println("broke bc target met");
            break;
        }

        // Update previous error and time for next loop
        linPID.prevError = linPID.error;
        headingPrevError = headingError;
        prevTime = currentTime;

        delay(10);
    }
    brake();
}

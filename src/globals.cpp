#include "globals.hpp"

// IMU state
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
static double headingZeroDeg = 0.0;

bool setupIMU() {
    if (!bno.begin()) {
        Serial.println("Failed to detect BNO055");
        return false;
    }
    bno.setExtCrystalUse(true);
    return true;
}

void calibrateHeadingZero() {
    sensors_event_t event;
    double accumulated = 0.0;
    const int samples = 20;
    for (int i = 0; i < samples; i++) {
        bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
        accumulated += event.orientation.x; // Heading in degrees
        delay(10);
    }
    headingZeroDeg = accumulated / samples;
    Serial.print("Heading zero set to: ");
    Serial.println(headingZeroDeg);
}

double readHeadingDeg() {
    sensors_event_t event;
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    double rawHeading = event.orientation.x; // 0-360 from IMU
    double relative = normalizeAngleDeg(rawHeading - headingZeroDeg);
    return relative;
}

double normalizeAngleDeg(double angleDeg) {
    while (angleDeg <= -180.0) angleDeg += 360.0;
    while (angleDeg > 180.0) angleDeg -= 360.0;
    return angleDeg;
}
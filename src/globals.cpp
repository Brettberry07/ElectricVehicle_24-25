#include "globals.hpp"

// MPU6050 functions and variables

// Sensitivity (for default ±250°/s full‐scale)
const float GYRO_SCALE = 131.0;  // LSB per °/s

// Calibration settings
const int CALIBRATION_SAMPLES = 50;
float gyroOffsetZ = 0;  // Calibration offset for z-axis gyro

float yaw = 0;          // Integrated yaw angle (in degrees)

unsigned long lastTime = 0;

// Function to read only the gyroscope's z-axis data from MPU6050
int16_t readGyroZ() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43 + 4); // 0x43 is the start of gyro data; skip first 4 bytes (gyro X and gyro Y)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t gz = Wire.read() << 8 | Wire.read();
    return gz;
}
  
// Function to calibrate the z-axis gyro offset
void calibrateGyroZ() {
    long sum = 0;
    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int16_t gz = readGyroZ();
        sum += gz;
        delay(3);
    }
    gyroOffsetZ = sum / (float)CALIBRATION_SAMPLES;
    Serial.print("Gyro Z offset (raw): ");
    Serial.println(gyroOffsetZ);
}
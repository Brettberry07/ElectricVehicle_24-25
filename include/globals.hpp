#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "EV.hpp"

// Defined pins
#define MOTOR_A1 10
#define MOTOR_A2 9
#define MOTOR_ENA 11

#define ENCODER_A1 2
#define ENCODER_A2 7

#define MOTOR_B1 5
#define MOTOR_B2 4
#define MOTOR_ENB 3

#define ENCODER_B1 8
#define ENCODER_B2 13

#define START_BUTTON 12

// mpu stuff
// I2C address of MPU6050 (AD0 tied to GND)
#define MPU_ADDR 0x68

extern const float GYRO_SCALE;  // LSB per °/s
extern const int CALIBRATION_SAMPLES;
extern float gyroOffsetZ;  // Calibration offset for z-axis gyro
extern float yaw;          // Integrated yaw angle (in degrees)
extern unsigned long lastTime;

// Function to read only the gyroscope's z-axis data from MPU6050
int16_t readGyroZ();
// Function to calibrate the z-axis gyro offset
void calibrateGyroZ();
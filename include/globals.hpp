#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

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

// IMU configuration
extern Adafruit_BNO055 bno;

// Initialize IMU and return success flag
bool setupIMU();

// Capture the current absolute heading as the zero reference
void calibrateHeadingZero();

// Read heading relative to zero reference in degrees (-180, 180]
double readHeadingDeg();

// Utility to keep angles bounded to (-180, 180]
double normalizeAngleDeg(double angleDeg);
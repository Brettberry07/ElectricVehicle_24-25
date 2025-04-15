#pragma once
#include "globals.hpp"

struct PIDConstants {
  double kP, kI, kD;
  double error = 0, prevError = 0, integral = 0, derivative = 0;
  double timeOut = 25;
  double low = -255, high = 150;

  PIDConstants(double p, double i, double d) : kP(p), kI(i), kD(d) {}
};

struct Position {
  double x, y;
  Position(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

struct SensorReadings {
  volatile int leftEncoderCount = 0, rightEncoderCount = 0;
};

class EV {
private:
    static EV* instance; // Static instance for interrupt handling

    int pinA1, pinA2, pinENA, pinB1, pinB2, pinENB;
    

    Position pos;
    PIDConstants linPID;

    // void updateEncoder(int &encoderCount, int pinA, int pinB);

public:
    EV(
        int p1, int p2, int pA, int p3, int p4, int pB, 
        int pSA1, int pSA2, int pSB1, int pSB2, 
        int pinButn
    );

    SensorReadings sensor;
    int pinSA1, pinSA2, pinSB1, pinSB2, pinButton;

    static const double distOneTick;
    
    void initialize();
    void forward(uint8_t speed1, uint8_t speed2);
    void backward(uint8_t speed);
    void left(uint8_t speed);
    void right(uint8_t speed);
    void brake();
    
    double getDistance();
    
    static void getSensorsOnInterupt();  // Make static
    void tarePosition();
    void updateEncoder(volatile int &encoderCount, int pinA, int pinB);

    double getLinearError(double dist, double goal);
    double getAngularError(SensorReadings pos, double goal = 0);

    void compLoop(double goal);
    void PIDLoop(double goal);
    void updateOdom();
};
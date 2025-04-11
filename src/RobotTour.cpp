// #include <Romi32U4.h>
// #include <Wire.h>
// #include <LSM6.h>

// Romi32U4Motors motors;
// Romi32U4Encoders encoders;
// Romi32U4ButtonC buttonC;
// LSM6 imu; 

// #define M_PI 3.14159265358979323846

// //Specs
// double diameter = 7;
// double circumference = M_PI * diameter;
// int motor_PPR = 12;
// int gearRatio = 120;
// int PPR = motor_PPR * gearRatio;
// double wheelbase = 14.2;
// double wheelbaseCircumference = M_PI * wheelbase;

// double DistancePerTick = circumference / PPR;
// double DegreesPerTick = (360 * DistancePerTick) / wheelbaseCircumference;

// float GYRO_SENSITIVITY = 0.075; 

// void calibrateGyro() { // make sure you haven't moved robot when you run this
//     Serial.println("Calibrating gyro...");
//     long biasSum = 0;
//     int numSamples = 500;

//     for (int i = 0; i < numSamples; i++) {
//         imu.read();
//         biasSum += imu.g.z;
//         delay(2);
//     }

//     float gyroBias = biasSum / (float)numSamples;
//     Serial.print("Gyro Bias: ");
//     Serial.println(gyroBias);

//     GYRO_SENSITIVITY = gyroBias; // Store the bias for correction
// }

// void setup() {
//     Serial.begin(9600);
//     while (!Serial);
//     Wire.begin();

//     Serial.println("Setup started");  

//     if (!imu.init()) {
//         Serial.println("couldnt detect imu");
//         while (1);
//     }

//     imu.enableDefault();
//     imu.writeReg(LSM6::CTRL2_G, 0x4C);  

// }

// // void turn(double degrees) {
// //     imu.read();
// //     float angle = 0;
// //     unsigned long prevTime = millis();

// //     int turnDirection = (degrees > 0) ? 1 : -1; 
// //     degrees = abs(degrees);  

// //     motors.setSpeeds(turnDirection * 100, -turnDirection * 100);

// //     while (abs(angle) < degrees) {
// //         imu.read();
// //         unsigned long currentTime = millis();
// //         float deltaTime = (currentTime - prevTime) / 1000.0;  
// //         prevTime = currentTime;

// //         float gyroZ = imu.g.z * GYRO_SENSITIVITY;  
// //         angle += gyroZ * deltaTime;
// //     }

// //     motors.setSpeeds(0, 0);
// //     delay(100);
// // }

// void turn(double degrees) {
//     imu.read();
//     float angle = 0;
//     unsigned long prevTime = micros();  // Higher resolution timing

//     int turnDirection = (degrees > 0) ? 1 : -1; 
//     degrees = abs(degrees);  

//     motors.setSpeeds(turnDirection * 100, -turnDirection * 100);

//     while (abs(angle) < degrees) {
//         imu.read();
//         unsigned long currentTime = micros();
//         float deltaTime = (currentTime - prevTime) / 1000000.0;  // Convert to seconds
//         prevTime = currentTime;

//         float gyroZ = (imu.g.z - GYRO_SENSITIVITY) * 0.00875; // Apply bias correction, scale for degrees/sec
//         angle += gyroZ * deltaTime;

//         // Slow down when close to target to reduce overshoot
//         if (abs(degrees - abs(angle)) < 10) {
//             motors.setSpeeds(turnDirection * 50, -turnDirection * 50);
//         }
//     }

//     motors.setSpeeds(0, 0);
//     delay(100);
// }

// void movePID(double setPoint, double percentage) {
//   long targetTicks = abs(setPoint) / DistancePerTick;
//   double maxSpeed_test = int((percentage/100) * 300);

//   encoders.getCountsAndResetLeft();  
//   encoders.getCountsAndResetRight();

//   //P Distance
//   const double kP = 0.20;

//   //PID drift
//   const double kPdrift = 0.5;
//   const double Kidrift = 0.1;
//   const double kDdrift = 0.05;

//   double integral = 0;
//   double prevError = 0;
//   const double integralLimit = 500;

//   int direction;

//   double powerL = 0;
//   double powerR = 0;

//   while (abs(encoders.getCountsLeft()) < targetTicks && abs(encoders.getCountsRight()) < targetTicks) {
//     long countL = encoders.getCountsLeft();
//     long countR = encoders.getCountsRight();

//     double avgCount = (countL + countR) / 2;

//     //P Distance
//     double error = targetTicks - avgCount;
//     double baseSpeed = error * kP;

//     baseSpeed = constrain(baseSpeed, 50, maxSpeed_test);

//     //PID Drift
//     double diff = countL - countR;

//     integral = integral + diff;
//     integral = constrain(integral, -integralLimit, integralLimit);

//     double derivative = diff - prevError;

//     double correction = diff * kPdrift + integral * Kidrift + derivative * kDdrift;

//     powerL = constrain(baseSpeed - correction, -200, 200);
//     powerR = constrain(baseSpeed + correction, -200, 200);

//     motors.setSpeeds(powerL, powerR);

//     prevError = diff;

//     Serial.print("diff: ");
//     Serial.println(diff);

//     delay(15);
//   }
//   motors.setSpeeds(0, 0);
// }

// void loop() {   
//     if (buttonC.isPressed()) { // 90° is 85°(vice versa), 180° is 182°
//         delay(1000);
//         turn(90);
//     }
// }
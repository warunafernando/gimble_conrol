/*
 * RobotDriver.h
 * 
 * Library for controlling Waveshare General Driver for Robots
 * Supports multiple motor and servo control
 */

#ifndef ROBOT_DRIVER_H
#define ROBOT_DRIVER_H

#include <Arduino.h>

class RobotDriver {
private:
  int motor1Pin;
  int motor2Pin;
  int motor3Pin;
  int motor4Pin;
  
  int motor1Speed;
  int motor2Speed;
  int motor3Speed;
  int motor4Speed;
  
  bool initialized;
  
public:
  // Constructor
  RobotDriver(int m1Pin, int m2Pin, int m3Pin, int m4Pin);
  
  // Initialize the driver
  void begin();
  
  // Set individual motor speed (-255 to 255)
  void setMotor1Speed(int speed);
  void setMotor2Speed(int speed);
  void setMotor3Speed(int speed);
  void setMotor4Speed(int speed);
  
  // Set all motor speeds at once
  void setMotorSpeeds(int m1, int m2, int m3, int m4);
  
  // Stop all motors
  void stopAll();
  
  // Update function (call in loop)
  void update();
  
  // Print current status
  void printStatus();
  
  // Get motor speeds
  int getMotor1Speed();
  int getMotor2Speed();
  int getMotor3Speed();
  int getMotor4Speed();
};

#endif

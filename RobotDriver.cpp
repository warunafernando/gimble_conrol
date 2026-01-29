/*
 * RobotDriver.cpp
 * 
 * Implementation of RobotDriver class
 */

#include "RobotDriver.h"

RobotDriver::RobotDriver(int m1Pin, int m2Pin, int m3Pin, int m4Pin) {
  this->motor1Pin = m1Pin;
  this->motor2Pin = m2Pin;
  this->motor3Pin = m3Pin;
  this->motor4Pin = m4Pin;
  this->motor1Speed = 0;
  this->motor2Speed = 0;
  this->motor3Speed = 0;
  this->motor4Speed = 0;
  this->initialized = false;
}

void RobotDriver::begin() {
  // Set motor pins as outputs
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);
  
  // Initialize all motors to stopped
  analogWrite(motor1Pin, 0);
  analogWrite(motor2Pin, 0);
  analogWrite(motor3Pin, 0);
  analogWrite(motor4Pin, 0);
  
  initialized = true;
}

void RobotDriver::setMotor1Speed(int speed) {
  motor1Speed = constrain(speed, -255, 255);
  int pwmValue = abs(motor1Speed);
  analogWrite(motor1Pin, pwmValue);
}

void RobotDriver::setMotor2Speed(int speed) {
  motor2Speed = constrain(speed, -255, 255);
  int pwmValue = abs(motor2Speed);
  analogWrite(motor2Pin, pwmValue);
}

void RobotDriver::setMotor3Speed(int speed) {
  motor3Speed = constrain(speed, -255, 255);
  int pwmValue = abs(motor3Speed);
  analogWrite(motor3Pin, pwmValue);
}

void RobotDriver::setMotor4Speed(int speed) {
  motor4Speed = constrain(speed, -255, 255);
  int pwmValue = abs(motor4Speed);
  analogWrite(motor4Pin, pwmValue);
}

void RobotDriver::setMotorSpeeds(int m1, int m2, int m3, int m4) {
  setMotor1Speed(m1);
  setMotor2Speed(m2);
  setMotor3Speed(m3);
  setMotor4Speed(m4);
}

void RobotDriver::stopAll() {
  setMotorSpeeds(0, 0, 0, 0);
}

void RobotDriver::update() {
  // Update motor speeds if needed
  // This can be used for acceleration/deceleration in the future
}

void RobotDriver::printStatus() {
  Serial.println("Motor Status:");
  Serial.print("  Motor 1: "); Serial.println(motor1Speed);
  Serial.print("  Motor 2: "); Serial.println(motor2Speed);
  Serial.print("  Motor 3: "); Serial.println(motor3Speed);
  Serial.print("  Motor 4: "); Serial.println(motor4Speed);
}

int RobotDriver::getMotor1Speed() {
  return motor1Speed;
}

int RobotDriver::getMotor2Speed() {
  return motor2Speed;
}

int RobotDriver::getMotor3Speed() {
  return motor3Speed;
}

int RobotDriver::getMotor4Speed() {
  return motor4Speed;
}

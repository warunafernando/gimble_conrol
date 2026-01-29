/*
 * PanTiltController.h
 * 
 * Library for controlling 2-axis pan-tilt camera module
 * Compatible with Waveshare Pan-Tilt Camera Module
 */

#ifndef PANTILT_CONTROLLER_H
#define PANTILT_CONTROLLER_H

#include <Servo.h>
#include <Arduino.h>

class PanTiltController {
private:
  Servo panServo;
  Servo tiltServo;
  int panPin;
  int tiltPin;
  int currentPan;
  int currentTilt;
  int targetPan;
  int targetTilt;
  bool smoothMovement;
  int movementSpeed;  // Steps per update for smooth movement
  
public:
  // Constructor
  PanTiltController(int panPin, int tiltPin);
  
  // Initialize the controller
  void begin();
  
  // Set pan angle (0-180 degrees)
  void setPan(int angle);
  
  // Set tilt angle (0-180 degrees)
  void setTilt(int angle);
  
  // Set both pan and tilt angles
  void setPosition(int pan, int tilt);
  
  // Enable/disable smooth movement
  void setSmoothMovement(bool enable, int speed = 2);
  
  // Get current pan angle
  int getPan();
  
  // Get current tilt angle
  int getTilt();
  
  // Update function (call in loop for smooth movement)
  void update();
  
  // Center the camera
  void center();
  
  // Move relative to current position
  void moveRelative(int panDelta, int tiltDelta);
};

#endif

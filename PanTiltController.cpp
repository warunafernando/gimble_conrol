/*
 * PanTiltController.cpp
 * 
 * Implementation of PanTiltController class
 */

#include "PanTiltController.h"

PanTiltController::PanTiltController(int panPin, int tiltPin) {
  this->panPin = panPin;
  this->tiltPin = tiltPin;
  this->currentPan = 90;
  this->currentTilt = 90;
  this->targetPan = 90;
  this->targetTilt = 90;
  this->smoothMovement = false;
  this->movementSpeed = 2;
}

void PanTiltController::begin() {
  panServo.attach(panPin);
  tiltServo.attach(tiltPin);
  
  // Center servos on startup
  panServo.write(90);
  tiltServo.write(90);
  
  delay(500); // Give servos time to reach position
}

void PanTiltController::setPan(int angle) {
  angle = constrain(angle, 0, 180);
  targetPan = angle;
  
  if (!smoothMovement) {
    currentPan = angle;
    panServo.write(angle);
  }
}

void PanTiltController::setTilt(int angle) {
  angle = constrain(angle, 0, 180);
  targetTilt = angle;
  
  if (!smoothMovement) {
    currentTilt = angle;
    tiltServo.write(angle);
  }
}

void PanTiltController::setPosition(int pan, int tilt) {
  setPan(pan);
  setTilt(tilt);
}

void PanTiltController::setSmoothMovement(bool enable, int speed) {
  smoothMovement = enable;
  movementSpeed = constrain(speed, 1, 10);
}

int PanTiltController::getPan() {
  return currentPan;
}

int PanTiltController::getTilt() {
  return currentTilt;
}

void PanTiltController::update() {
  if (smoothMovement) {
    // Smooth pan movement
    if (currentPan < targetPan) {
      currentPan = min(currentPan + movementSpeed, targetPan);
      panServo.write(currentPan);
    } else if (currentPan > targetPan) {
      currentPan = max(currentPan - movementSpeed, targetPan);
      panServo.write(currentPan);
    }
    
    // Smooth tilt movement
    if (currentTilt < targetTilt) {
      currentTilt = min(currentTilt + movementSpeed, targetTilt);
      tiltServo.write(currentTilt);
    } else if (currentTilt > targetTilt) {
      currentTilt = max(currentTilt - movementSpeed, targetTilt);
      tiltServo.write(currentTilt);
    }
  }
}

void PanTiltController::center() {
  setPosition(90, 90);
}

void PanTiltController::moveRelative(int panDelta, int tiltDelta) {
  int newPan = constrain(currentPan + panDelta, 0, 180);
  int newTilt = constrain(currentTilt + tiltDelta, 0, 180);
  setPosition(newPan, newTilt);
}

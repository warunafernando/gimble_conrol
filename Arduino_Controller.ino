/*
 * Arduino Controller for Waveshare 2-Axis Pan-Tilt Camera Module
 * and General Driver for Robots
 * 
 * This sketch controls a 2-axis pan-tilt camera system using
 * the Waveshare General Driver for Robots board.
 */

#include <Servo.h>
#include "PanTiltController.h"
#include "RobotDriver.h"

// Pin definitions for General Driver for Robots
// Adjust these based on your specific wiring
#define PAN_SERVO_PIN 9      // Pan servo control pin
#define TILT_SERVO_PIN 10    // Tilt servo control pin
#define MOTOR1_PIN 5         // Motor 1 control pin
#define MOTOR2_PIN 6         // Motor 2 control pin
#define MOTOR3_PIN 7         // Motor 3 control pin
#define MOTOR4_PIN 8         // Motor 4 control pin

// Serial communication settings
#define SERIAL_BAUD 115200

// Create controller instances
PanTiltController panTilt(PAN_SERVO_PIN, TILT_SERVO_PIN);
RobotDriver robotDriver(MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN);

// Control variables
int panAngle = 90;    // Pan angle (0-180 degrees, 90 = center)
int tiltAngle = 90;   // Tilt angle (0-180 degrees, 90 = center)

void setup() {
  // Initialize serial communication
  Serial.begin(SERIAL_BAUD);
  Serial.println("Waveshare Pan-Tilt Camera Controller");
  Serial.println("Initializing...");
  
  // Initialize pan-tilt controller
  panTilt.begin();
  Serial.println("Pan-Tilt Controller initialized");
  
  // Initialize robot driver
  robotDriver.begin();
  Serial.println("Robot Driver initialized");
  
  // Center the camera
  panTilt.setPosition(90, 90);
  Serial.println("Camera centered");
  Serial.println("Ready for commands!");
  Serial.println("Commands:");
  Serial.println("  P<angle> - Set pan angle (0-180)");
  Serial.println("  T<angle> - Set tilt angle (0-180)");
  Serial.println("  M<speed1>,<speed2>,<speed3>,<speed4> - Set motor speeds");
  Serial.println("  C - Center camera");
  Serial.println("  S - Get status");
}

void loop() {
  // Check for serial commands
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }
  
  // Update controllers
  panTilt.update();
  robotDriver.update();
  
  delay(10); // Small delay for stability
}

void processCommand(String command) {
  if (command.length() == 0) return;
  
  char cmd = command.charAt(0);
  command = command.substring(1);
  
  switch (cmd) {
    case 'P':
    case 'p':
      // Set pan angle
      panAngle = constrain(command.toInt(), 0, 180);
      panTilt.setPan(panAngle);
      Serial.print("Pan set to: ");
      Serial.println(panAngle);
      break;
      
    case 'T':
    case 't':
      // Set tilt angle
      tiltAngle = constrain(command.toInt(), 0, 180);
      panTilt.setTilt(tiltAngle);
      Serial.print("Tilt set to: ");
      Serial.println(tiltAngle);
      break;
      
    case 'M':
    case 'm':
      // Set motor speeds
      // Format: M<speed1>,<speed2>,<speed3>,<speed4>
      int speeds[4];
      int index = 0;
      int lastIndex = 0;
      
      for (int i = 0; i < command.length() && index < 4; i++) {
        if (command.charAt(i) == ',' || i == command.length() - 1) {
          speeds[index] = command.substring(lastIndex, i == command.length() - 1 ? i + 1 : i).toInt();
          speeds[index] = constrain(speeds[index], -255, 255);
          lastIndex = i + 1;
          index++;
        }
      }
      
      robotDriver.setMotorSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]);
      Serial.print("Motors set to: ");
      Serial.print(speeds[0]); Serial.print(", ");
      Serial.print(speeds[1]); Serial.print(", ");
      Serial.print(speeds[2]); Serial.print(", ");
      Serial.println(speeds[3]);
      break;
      
    case 'C':
    case 'c':
      // Center camera
      panAngle = 90;
      tiltAngle = 90;
      panTilt.setPosition(90, 90);
      Serial.println("Camera centered");
      break;
      
    case 'S':
    case 's':
      // Get status
      Serial.println("=== Status ===");
      Serial.print("Pan: "); Serial.println(panAngle);
      Serial.print("Tilt: "); Serial.println(tiltAngle);
      robotDriver.printStatus();
      break;
      
    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd);
      break;
  }
}

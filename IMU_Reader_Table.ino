/*
 * IMU Data Reader - Table Format
 * 
 * Displays IMU data in a formatted table with headers
 * 
 * Hardware: ESP32 (Waveshare General Driver for Robots)
 * Sensors: QMI8658 @ 0x6B, AK09918 @ 0x0C
 */

#include "IMU.h"
#include <Wire.h>

EulerAngles angles;
IMU_ST_SENSOR_DATA_FLOAT gyroData;
IMU_ST_SENSOR_DATA_FLOAT accelData;
IMU_ST_SENSOR_DATA magData;

unsigned long lastUpdate = 0;
const unsigned long UPDATE_RATE = 200; // Update every 200ms (5 Hz for table format)
bool firstPrint = true;

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════════════════════════════╗");
  Serial.println("║                        IMU Data Reader - Table Format                          ║");
  Serial.println("╚═══════════════════════════════════════════════════════════════════════════════╝");
  Serial.println();
  
  Serial.println("Initializing I2C and IMU sensors...");
  Wire.begin();
  delay(100);
  
  imuInit();
  Serial.println("IMU initialized successfully!");
  Serial.println();
  Serial.println("Starting data acquisition...");
  delay(1000);
  
  // Print table header
  printTableHeader();
  
  lastUpdate = millis();
}

void loop() {
  imuDataGet(&angles, &gyroData, &accelData, &magData);
  
  if (millis() - lastUpdate >= UPDATE_RATE) {
    printTableRow();
    lastUpdate = millis();
  }
  
  delay(10);
}

void printTableHeader() {
  Serial.println("┌──────────┬──────────┬──────────┬──────────────┬──────────────┬──────────────┬──────────────┬──────────────┬──────────────┐");
  Serial.println("│   Roll   │  Pitch   │   Yaw    │   Accel X    │   Accel Y    │   Accel Z    │   Gyro X     │   Gyro Y     │   Gyro Z     │");
  Serial.println("│   (°)    │   (°)    │   (°)    │   (m/s²)     │   (m/s²)     │   (m/s²)     │   (rad/s)    │   (rad/s)    │   (rad/s)    │");
  Serial.println("├──────────┼──────────┼──────────┼──────────────┼──────────────┼──────────────┼──────────────┼──────────────┼──────────────┤");
}

void printTableRow() {
  Serial.print("│ ");
  Serial.print(angles.roll, 6);
  Serial.print(" │ ");
  Serial.print(angles.pitch, 6);
  Serial.print(" │ ");
  Serial.print(angles.yaw, 6);
  Serial.print(" │ ");
  Serial.print(accelData.X, 6);
  Serial.print(" │ ");
  Serial.print(accelData.Y, 6);
  Serial.print(" │ ");
  Serial.print(accelData.Z, 6);
  Serial.print(" │ ");
  Serial.print(gyroData.X, 6);
  Serial.print(" │ ");
  Serial.print(gyroData.Y, 6);
  Serial.print(" │ ");
  Serial.print(gyroData.Z, 6);
  Serial.println(" │");
}

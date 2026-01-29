/*
 * IMU Data Reader
 * 
 * Simple program to read and display data from QMI8658 (6-axis IMU) 
 * and AK09918 (3-axis magnetometer) sensors.
 * 
 * Hardware: ESP32 with Waveshare General Driver for Robots
 * Sensors: QMI8658 (0x6B) + AK09918 (0x0C) on I2C bus
 */

#include "IMU.h"
#include <Wire.h>

// Global variables for IMU data
EulerAngles stAngles;
IMU_ST_SENSOR_DATA_FLOAT stGyroRawData;
IMU_ST_SENSOR_DATA_FLOAT stAccelRawData;
IMU_ST_SENSOR_DATA stMagnRawData;

// Display formatting
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 100; // Print every 100ms (10 Hz)

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("   IMU Data Reader");
  Serial.println("========================================");
  Serial.println("Initializing I2C bus...");
  
  // Initialize I2C (default pins: SDA=21, SCL=22 for ESP32)
  Wire.begin();
  delay(100);
  
  Serial.println("Initializing IMU sensors...");
  Serial.println("  - QMI8658 (6-axis: Accel + Gyro)");
  Serial.println("  - AK09918 (3-axis: Magnetometer)");
  Serial.println();
  
  // Initialize IMU
  imuInit();
  
  Serial.println("IMU initialization complete!");
  Serial.println("Starting data read loop...");
  Serial.println();
  Serial.println("Format: Roll, Pitch, Yaw | Accel (X,Y,Z) | Gyro (X,Y,Z) | Mag (X,Y,Z)");
  Serial.println("----------------------------------------------------------------------------");
  
  lastPrintTime = millis();
}

void loop() {
  // Read IMU data
  imuDataGet(&stAngles, &stGyroRawData, &stAccelRawData, &stMagnRawData);
  
  // Print data at specified interval
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    printIMUData();
    lastPrintTime = currentTime;
  }
  
  // Small delay to prevent overwhelming the I2C bus
  delay(5);
}

void printIMUData() {
  // Print Euler angles (Roll, Pitch, Yaw)
  Serial.print("RPY: ");
  Serial.print(stAngles.roll, 2);
  Serial.print("°, ");
  Serial.print(stAngles.pitch, 2);
  Serial.print("°, ");
  Serial.print(stAngles.yaw, 2);
  Serial.print("°  |  ");
  
  // Print Accelerometer data (m/s²)
  Serial.print("Acc: ");
  Serial.print(stAccelRawData.X, 3);
  Serial.print(", ");
  Serial.print(stAccelRawData.Y, 3);
  Serial.print(", ");
  Serial.print(stAccelRawData.Z, 3);
  Serial.print(" m/s²  |  ");
  
  // Print Gyroscope data (rad/s)
  Serial.print("Gyro: ");
  Serial.print(stGyroRawData.X, 3);
  Serial.print(", ");
  Serial.print(stGyroRawData.Y, 3);
  Serial.print(", ");
  Serial.print(stGyroRawData.Z, 3);
  Serial.print(" rad/s  |  ");
  
  // Print Magnetometer data (uT)
  Serial.print("Mag: ");
  Serial.print(stMagnRawData.s16X);
  Serial.print(", ");
  Serial.print(stMagnRawData.s16Y);
  Serial.print(", ");
  Serial.print(stMagnRawData.s16Z);
  Serial.println(" uT");
}

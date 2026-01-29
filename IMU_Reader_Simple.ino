/*
 * IMU Data Reader - Simple Version
 * 
 * Reads and displays data from QMI8658 (6-axis IMU) and AK09918 (magnetometer)
 * 
 * Hardware: ESP32 (Waveshare General Driver for Robots)
 * Sensors: QMI8658 @ 0x6B, AK09918 @ 0x0C
 * 
 * Usage:
 *   1. Upload this sketch to ESP32
 *   2. Open Serial Monitor at 115200 baud
 *   3. View real-time IMU data
 */

#include "IMU.h"
#include <Wire.h>

// Data structures
EulerAngles angles;
IMU_ST_SENSOR_DATA_FLOAT gyroData;
IMU_ST_SENSOR_DATA_FLOAT accelData;
IMU_ST_SENSOR_DATA magData;

// Timing
unsigned long lastUpdate = 0;
const unsigned long UPDATE_RATE = 50; // Update every 50ms (20 Hz)

void setup() {
  Serial.begin(115200);
  delay(2000); // Wait for serial connection
  
  Serial.println("\n");
  Serial.println("╔═══════════════════════════════════════════════════════╗");
  Serial.println("║           IMU Data Reader - ESP32                     ║");
  Serial.println("╚═══════════════════════════════════════════════════════╝");
  Serial.println();
  
  Serial.println("[INFO] Initializing I2C bus...");
  Wire.begin();
  delay(100);
  
  Serial.println("[INFO] Initializing sensors:");
  Serial.println("       - QMI8658: 6-axis IMU (Accelerometer + Gyroscope)");
  Serial.println("       - AK09918: 3-axis Magnetometer");
  Serial.println();
  
  // Initialize IMU
  Serial.print("[INFO] Initializing IMU... ");
  imuInit();
  Serial.println("Done!");
  Serial.println();
  
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println("  Real-time IMU Data (Update rate: 20 Hz)");
  Serial.println("═══════════════════════════════════════════════════════════");
  Serial.println();
  
  lastUpdate = millis();
}

void loop() {
  // Read IMU data
  imuDataGet(&angles, &gyroData, &accelData, &magData);
  
  // Print at specified rate
  if (millis() - lastUpdate >= UPDATE_RATE) {
    printFormattedData();
    lastUpdate = millis();
  }
  
  delay(1);
}

void printFormattedData() {
  // Clear previous line (for cleaner output)
  Serial.print("\r");
  
  // Euler Angles (Roll, Pitch, Yaw)
  Serial.print("RPY: [");
  Serial.print(angles.roll, 1);
  Serial.print("°, ");
  Serial.print(angles.pitch, 1);
  Serial.print("°, ");
  Serial.print(angles.yaw, 1);
  Serial.print("°]  ");
  
  // Accelerometer (m/s²)
  Serial.print("Acc: [");
  Serial.print(accelData.X, 2);
  Serial.print(", ");
  Serial.print(accelData.Y, 2);
  Serial.print(", ");
  Serial.print(accelData.Z, 2);
  Serial.print("] m/s²  ");
  
  // Gyroscope (rad/s)
  Serial.print("Gyro: [");
  Serial.print(gyroData.X, 3);
  Serial.print(", ");
  Serial.print(gyroData.Y, 3);
  Serial.print(", ");
  Serial.print(gyroData.Z, 3);
  Serial.print("] rad/s  ");
  
  // Magnetometer (uT)
  Serial.print("Mag: [");
  Serial.print(magData.s16X);
  Serial.print(", ");
  Serial.print(magData.s16Y);
  Serial.print(", ");
  Serial.print(magData.s16Z);
  Serial.print("] uT");
  
  // Add spaces to clear any remaining characters
  Serial.print("                    ");
}

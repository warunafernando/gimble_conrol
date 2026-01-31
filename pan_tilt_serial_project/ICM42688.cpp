// ICM42688 6-axis IMU Driver Implementation

#include "ICM42688.h"

ICM42688::ICM42688(uint8_t addr) : _addr(addr) {
    _gyroFS = ICM42688_GYRO_FS_2000DPS;
    _accelFS = ICM42688_ACCEL_FS_16G;
    _accelX = _accelY = _accelZ = 0;
    _gyroX = _gyroY = _gyroZ = 0;
    _temp = 0;
}

bool ICM42688::begin() {
    delay(20);
    
    Wire.beginTransmission(_addr);
    if (Wire.endTransmission() != 0) {
        uint8_t altAddr = (_addr == ICM42688_ADDR_HIGH) ? ICM42688_ADDR_LOW : ICM42688_ADDR_HIGH;
        Wire.beginTransmission(altAddr);
        if (Wire.endTransmission() == 0) {
            _addr = altAddr;
        } else {
            return false;  // No device at either address
        }
    }
    
    // ICM42688: Select Bank 0 FIRST - required before any register access
    writeReg(ICM42688_REG_BANK_SEL, 0x00);
    delay(5);
    
    uint8_t id = getDeviceID();
    // ICM-42688-P = 0x47, ICM-42688-V = 0x42, ICM-42605 = 0x6F
    if (id != 0x47 && id != 0x42 && id != 0x6F) {
        // Not ICM42688 - might be ICM20948 (0xEA) or other; reject
        return false;
    }
    
    // Soft reset
    writeReg(ICM42688_REG_DEVICE_CONFIG, 0x01);
    delay(10);  // Short delay after reset
    
    // Select Bank 0 after reset
    writeReg(ICM42688_REG_BANK_SEL, 0x00);
    delay(10);
    
    // Enable gyro and accel in Low Noise mode FIRST (before configuring)
    // Bits [3:2] = GYRO_MODE (11 = Low Noise), Bits [1:0] = ACCEL_MODE (11 = Low Noise)
    writeReg(ICM42688_REG_PWR_MGMT0, 0x0F);
    delay(200);  // CRITICAL: Wait for sensors to power up and stabilize (increased from 50ms)
    
    // Configure gyroscope: 2000 dps, 1kHz ODR
    writeReg(ICM42688_REG_GYRO_CONFIG0, (ICM42688_GYRO_FS_2000DPS << 5) | ICM42688_ODR_1KHZ);
    _gyroFS = ICM42688_GYRO_FS_2000DPS;
    delay(10);
    
    // Configure accelerometer: 16g, 1kHz ODR
    writeReg(ICM42688_REG_ACCEL_CONFIG0, (ICM42688_ACCEL_FS_16G << 5) | ICM42688_ODR_1KHZ);
    _accelFS = ICM42688_ACCEL_FS_16G;
    delay(10);
    
    return true;
}

bool ICM42688::isConnected() {
    Wire.beginTransmission(_addr);
    return (Wire.endTransmission() == 0);
}

uint8_t ICM42688::getDeviceID() {
    return readReg(ICM42688_REG_WHO_AM_I);
}

void ICM42688::setGyroFS(uint8_t fs) {
    _gyroFS = fs;
    uint8_t config = readReg(ICM42688_REG_GYRO_CONFIG0);
    config = (config & 0x1F) | (fs << 5);
    writeReg(ICM42688_REG_GYRO_CONFIG0, config);
}

void ICM42688::setAccelFS(uint8_t fs) {
    _accelFS = fs;
    uint8_t config = readReg(ICM42688_REG_ACCEL_CONFIG0);
    config = (config & 0x1F) | (fs << 5);
    writeReg(ICM42688_REG_ACCEL_CONFIG0, config);
}

void ICM42688::setGyroODR(uint8_t odr) {
    uint8_t config = readReg(ICM42688_REG_GYRO_CONFIG0);
    config = (config & 0xF0) | (odr & 0x0F);
    writeReg(ICM42688_REG_GYRO_CONFIG0, config);
}

void ICM42688::setAccelODR(uint8_t odr) {
    uint8_t config = readReg(ICM42688_REG_ACCEL_CONFIG0);
    config = (config & 0xF0) | (odr & 0x0F);
    writeReg(ICM42688_REG_ACCEL_CONFIG0, config);
}

void ICM42688::readAll() {
    // Small delay to ensure sensor has fresh data
    delay(2);
    
    uint8_t buf[14];
    memset(buf, 0, sizeof(buf));
    readRegs(ICM42688_REG_TEMP_DATA1, buf, 14);
    
    _temp = ((int16_t)buf[0] << 8) | buf[1];
    _accelX = ((int16_t)buf[2] << 8) | buf[3];
    _accelY = ((int16_t)buf[4] << 8) | buf[5];
    _accelZ = ((int16_t)buf[6] << 8) | buf[7];
    _gyroX = ((int16_t)buf[8] << 8) | buf[9];
    _gyroY = ((int16_t)buf[10] << 8) | buf[11];
    _gyroZ = ((int16_t)buf[12] << 8) | buf[13];
    
}

float ICM42688::getGyroScale() {
    switch (_gyroFS) {
        case ICM42688_GYRO_FS_2000DPS:  return 2000.0f / 32768.0f;
        case ICM42688_GYRO_FS_1000DPS:  return 1000.0f / 32768.0f;
        case ICM42688_GYRO_FS_500DPS:   return 500.0f / 32768.0f;
        case ICM42688_GYRO_FS_250DPS:   return 250.0f / 32768.0f;
        case ICM42688_GYRO_FS_125DPS:   return 125.0f / 32768.0f;
        case ICM42688_GYRO_FS_62_5DPS:  return 62.5f / 32768.0f;
        case ICM42688_GYRO_FS_31_25DPS: return 31.25f / 32768.0f;
        case ICM42688_GYRO_FS_15_625DPS:return 15.625f / 32768.0f;
        default: return 2000.0f / 32768.0f;
    }
}

float ICM42688::getAccelScale() {
    switch (_accelFS) {
        case ICM42688_ACCEL_FS_16G: return 16.0f / 32768.0f;
        case ICM42688_ACCEL_FS_8G:  return 8.0f / 32768.0f;
        case ICM42688_ACCEL_FS_4G:  return 4.0f / 32768.0f;
        case ICM42688_ACCEL_FS_2G:  return 2.0f / 32768.0f;
        default: return 16.0f / 32768.0f;
    }
}

float ICM42688::getAccelX() { return _accelX * getAccelScale(); }
float ICM42688::getAccelY() { return _accelY * getAccelScale(); }
float ICM42688::getAccelZ() { return _accelZ * getAccelScale(); }
float ICM42688::getGyroX() { return _gyroX * getGyroScale(); }
float ICM42688::getGyroY() { return _gyroY * getGyroScale(); }
float ICM42688::getGyroZ() { return _gyroZ * getGyroScale(); }

float ICM42688::getTemp() {
    // Temperature formula from datasheet: T = (TEMP_DATA / 132.48) + 25
    return (_temp / 132.48f) + 25.0f;
}

void ICM42688::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t ICM42688::readReg(uint8_t reg) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, (uint8_t)1);
    return Wire.read();
}

void ICM42688::readRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(_addr, len);
    for (uint8_t i = 0; i < len && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}

// ICM42688 6-axis IMU Driver (I2C)
// Accelerometer + Gyroscope

#ifndef ICM42688_H
#define ICM42688_H

#include <Arduino.h>
#include <Wire.h>

// I2C Addresses
#define ICM42688_ADDR_LOW  0x68  // AD0 = LOW
#define ICM42688_ADDR_HIGH 0x69  // AD0 = HIGH

// Register addresses
#define ICM42688_REG_DEVICE_CONFIG    0x11
#define ICM42688_REG_INT_CONFIG       0x14
#define ICM42688_REG_FIFO_CONFIG      0x16
#define ICM42688_REG_TEMP_DATA1       0x1D
#define ICM42688_REG_TEMP_DATA0       0x1E
#define ICM42688_REG_ACCEL_DATA_X1    0x1F
#define ICM42688_REG_ACCEL_DATA_X0    0x20
#define ICM42688_REG_ACCEL_DATA_Y1    0x21
#define ICM42688_REG_ACCEL_DATA_Y0    0x22
#define ICM42688_REG_ACCEL_DATA_Z1    0x23
#define ICM42688_REG_ACCEL_DATA_Z0    0x24
#define ICM42688_REG_GYRO_DATA_X1     0x25
#define ICM42688_REG_GYRO_DATA_X0     0x26
#define ICM42688_REG_GYRO_DATA_Y1     0x27
#define ICM42688_REG_GYRO_DATA_Y0     0x28
#define ICM42688_REG_GYRO_DATA_Z1     0x29
#define ICM42688_REG_GYRO_DATA_Z0     0x2A
#define ICM42688_REG_INT_STATUS       0x2D
#define ICM42688_REG_PWR_MGMT0        0x4E
#define ICM42688_REG_GYRO_CONFIG0     0x4F
#define ICM42688_REG_ACCEL_CONFIG0    0x50
#define ICM42688_REG_GYRO_CONFIG1     0x51
#define ICM42688_REG_ACCEL_CONFIG1    0x53
#define ICM42688_REG_WHO_AM_I         0x75
#define ICM42688_REG_BANK_SEL         0x76

// WHO_AM_I value
#define ICM42688_WHO_AM_I_VALUE       0x47

// Power management modes
#define ICM42688_PWR_GYRO_LN_MODE     0x0C  // Gyro Low Noise mode
#define ICM42688_PWR_ACCEL_LN_MODE    0x03  // Accel Low Noise mode
#define ICM42688_PWR_TEMP_ENABLE      0x00  // Temperature enabled

// Full scale ranges
#define ICM42688_GYRO_FS_2000DPS     0x00
#define ICM42688_GYRO_FS_1000DPS     0x01
#define ICM42688_GYRO_FS_500DPS      0x02
#define ICM42688_GYRO_FS_250DPS      0x03
#define ICM42688_GYRO_FS_125DPS      0x04
#define ICM42688_GYRO_FS_62_5DPS     0x05
#define ICM42688_GYRO_FS_31_25DPS    0x06
#define ICM42688_GYRO_FS_15_625DPS   0x07

#define ICM42688_ACCEL_FS_16G        0x00
#define ICM42688_ACCEL_FS_8G         0x01
#define ICM42688_ACCEL_FS_4G         0x02
#define ICM42688_ACCEL_FS_2G         0x03

// Output data rates
#define ICM42688_ODR_32KHZ           0x01
#define ICM42688_ODR_16KHZ           0x02
#define ICM42688_ODR_8KHZ            0x03
#define ICM42688_ODR_4KHZ            0x04
#define ICM42688_ODR_2KHZ            0x05
#define ICM42688_ODR_1KHZ            0x06
#define ICM42688_ODR_200HZ           0x07
#define ICM42688_ODR_100HZ           0x08
#define ICM42688_ODR_50HZ            0x09
#define ICM42688_ODR_25HZ            0x0A
#define ICM42688_ODR_12_5HZ          0x0B

class ICM42688 {
public:
    ICM42688(uint8_t addr = ICM42688_ADDR_HIGH);
    
    bool begin();
    bool isConnected();
    uint8_t getDeviceID();
    
    // Configuration
    void setGyroFS(uint8_t fs);
    void setAccelFS(uint8_t fs);
    void setGyroODR(uint8_t odr);
    void setAccelODR(uint8_t odr);
    
    // Data reading
    void readAll();
    
    // Raw data
    int16_t getAccelXRaw() { return _accelX; }
    int16_t getAccelYRaw() { return _accelY; }
    int16_t getAccelZRaw() { return _accelZ; }
    int16_t getGyroXRaw() { return _gyroX; }
    int16_t getGyroYRaw() { return _gyroY; }
    int16_t getGyroZRaw() { return _gyroZ; }
    int16_t getTempRaw() { return _temp; }
    
    // Scaled data (using current FS settings)
    float getAccelX();  // g
    float getAccelY();
    float getAccelZ();
    float getGyroX();   // dps
    float getGyroY();
    float getGyroZ();
    float getTemp();    // Celsius
    
private:
    uint8_t _addr;
    uint8_t _gyroFS;
    uint8_t _accelFS;
    
    int16_t _accelX, _accelY, _accelZ;
    int16_t _gyroX, _gyroY, _gyroZ;
    int16_t _temp;
    
    void writeReg(uint8_t reg, uint8_t value);
    uint8_t readReg(uint8_t reg);
    void readRegs(uint8_t reg, uint8_t* buf, uint8_t len);
    
    float getGyroScale();
    float getAccelScale();
};

#endif // ICM42688_H

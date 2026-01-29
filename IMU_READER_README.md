# IMU Data Reader Programs

Simple Arduino/ESP32 programs to read and display data from the IMU sensors (QMI8658 + AK09918) on the Waveshare General Driver for Robots board.

## Available Programs

### 1. `IMU_Reader.ino` - Basic Version
- Simple one-line output format
- Updates at 10 Hz (100ms intervals)
- Shows: Roll, Pitch, Yaw | Accelerometer | Gyroscope | Magnetometer

### 2. `IMU_Reader_Simple.ino` - Clean Output Version
- Single-line scrolling output (overwrites previous line)
- Updates at 20 Hz (50ms intervals)
- Nicely formatted with brackets and labels
- Recommended for most users

### 3. `IMU_Reader_Table.ino` - Table Format
- Formatted table with headers
- Updates at 5 Hz (200ms intervals)
- Best for data logging or detailed analysis

## Requirements

### Hardware
- ESP32 board (Waveshare General Driver for Robots)
- QMI8658 sensor (6-axis IMU) @ I2C address 0x6B
- AK09918 sensor (magnetometer) @ I2C address 0x0C
- I2C connection (default: SDA=GPIO21, SCL=GPIO22)

### Software
- Arduino IDE or PlatformIO
- Required libraries (should be in `pan_tilt_base_v0.9` folder):
  - `IMU.h` and `IMU.cpp`
  - `QMI8658.h` and `QMI8658.cpp`
  - `AK09918.h` and `AK09918.cpp`
  - `QMI8658reg.h`

## Setup Instructions

### Option 1: Using Arduino IDE

1. **Copy Required Files:**
   - Copy the `.ino` file you want to use
   - Ensure the IMU library files are accessible:
     - `IMU.h`, `IMU.cpp`
     - `QMI8658.h`, `QMI8658.cpp`, `QMI8658reg.h`
     - `AK09918.h`, `AK09918.cpp`

2. **Open in Arduino IDE:**
   - Open the `.ino` file
   - Select board: **ESP32 Dev Module**
   - Select correct COM port
   - Set upload speed if needed

3. **Upload:**
   - Click Upload button
   - Wait for compilation and upload

4. **View Data:**
   - Open Serial Monitor (Tools → Serial Monitor)
   - Set baud rate to **115200**
   - View real-time IMU data

### Option 2: Using PlatformIO

1. **Create Project Structure:**
   ```
   your_project/
   ├── src/
   │   └── main.cpp (copy .ino content here)
   ├── lib/
   │   └── IMU/ (copy IMU library files here)
   └── platformio.ini
   ```

2. **platformio.ini:**
   ```ini
   [env:esp32dev]
   platform = espressif32
   board = esp32dev
   framework = arduino
   monitor_speed = 115200
   ```

3. **Build and Upload:**
   ```bash
   pio run -t upload
   pio device monitor
   ```

## Output Format

### IMU_Reader_Simple.ino Example:
```
RPY: [2.5°, -1.2°, 45.8°]  Acc: [0.10, 0.00, 9.80] m/s²  Gyro: [0.001, 0.000, 0.000] rad/s  Mag: [1234, 567, 890] uT
```

### Data Explanation:

**Euler Angles (RPY):**
- **Roll**: Rotation around X-axis (degrees)
- **Pitch**: Rotation around Y-axis (degrees) - **Used for gimbal stabilization**
- **Yaw**: Rotation around Z-axis (degrees)

**Accelerometer (m/s²):**
- X, Y, Z: Linear acceleration in each axis
- At rest, Z should be ~9.8 m/s² (gravity)

**Gyroscope (rad/s):**
- X, Y, Z: Angular velocity in each axis
- At rest, all should be near 0

**Magnetometer (uT):**
- X, Y, Z: Magnetic field strength in microtesla
- Used for compass heading (yaw)

## Troubleshooting

### "qmi8658_init fail"
- Check I2C connections (SDA, SCL)
- Verify sensor is powered
- Check I2C address (should be 0x6B)
- Try different I2C pins if needed

### "AK09918_init fail"
- Check I2C connections
- Verify sensor is powered
- Check I2C address (should be 0x0C)
- Program will retry up to 10 times automatically

### No Data or Garbage Data
- Check Serial Monitor baud rate (115200)
- Verify I2C bus speed
- Check for I2C bus conflicts
- Ensure sensors are properly initialized

### Compilation Errors
- Ensure all library files are in the correct location
- Check that `Wire.h` is available (standard Arduino library)
- Verify ESP32 board package is installed

## Customization

### Change Update Rate
Modify the `UPDATE_RATE` or `PRINT_INTERVAL` constant:
```cpp
const unsigned long UPDATE_RATE = 100; // milliseconds
```

### Change I2C Pins
If using different I2C pins, modify in `IMU.cpp`:
```cpp
Wire.begin(SDA_PIN, SCL_PIN);
```

### Add Data Logging
Add SD card or file writing to log data:
```cpp
// Example: Log to Serial in CSV format
Serial.print(millis());
Serial.print(",");
Serial.print(angles.roll);
Serial.print(",");
Serial.println(angles.pitch);
```

## Integration with Main Firmware

These programs are standalone for testing. To integrate IMU reading into the main firmware:

1. Use `imu_init()` in `setup()`
2. Call `updateIMUData()` in `loop()`
3. Access global variables: `icm_roll`, `icm_pitch`, `icm_yaw`, etc.

## References

- Main firmware: `pan_tilt_base_v0.9/`
- IMU library: `pan_tilt_base_v0.9/IMU.h`, `IMU.cpp`
- Sensor datasheets:
  - QMI8658: 6-axis motion sensor
  - AK09918: 3-axis magnetometer

## Notes

- IMU data is updated every loop iteration in main firmware (~100-200 Hz)
- AHRS algorithm processes at ~41.67 Hz (halfT = 0.024s)
- Magnetometer samples at 100 Hz
- For gimbal stabilization, primarily `icm_pitch` is used

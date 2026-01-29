# IMU Data Monitor

Python program to read and display IMU data from the ESP32 Pan-Tilt system via USB serial.

## Requirements

- Python 3.6+
- pyserial library (already in `requirements.txt`)

## Installation

```bash
python -m pip install -r requirements.txt
```

## Usage

### Single Read

Get IMU data once:

```bash
python imu_monitor.py --port COM7
```

### Continuous Mode

Continuously display IMU data (updates at specified rate):

```bash
python imu_monitor.py --port COM7 --continuous
python imu_monitor.py --port COM7 --continuous --rate 10
```

### Table Format

Display data in a formatted table:

```bash
python imu_monitor.py --port COM7 --table
python imu_monitor.py --port COM7 --table --rate 2
```

## Command Line Options

- `--port PORT` (required): Serial port (e.g., `COM7` on Windows, `/dev/ttyUSB0` on Linux)
- `--baud RATE`: Baud rate (default: 115200)
- `--continuous`: Continuously display IMU data
- `--table`: Display in table format (implies continuous mode)
- `--rate HZ`: Update rate in Hz for continuous mode (default: 5.0)

## Output Format

### Single/Continuous Mode

```
[14:23:45.123] IMU Data:
  Euler Angles:  Roll=   2.50°  Pitch=  -1.20°  Yaw=  45.80°
  Accelerometer: X=  0.100  Y=  0.000  Z=  9.800 m/s²
  Gyroscope:     X=  0.001  Y=  0.000  Z=  0.000 rad/s
  Magnetometer:  X=   1234  Y=    567  Z=    890 uT
  Temperature:   25.5°C
```

### Table Format

```
Time         Roll     Pitch    Yaw      AccX     AccY     AccZ     GyroX    GyroY    GyroZ    MagX    MagY    MagZ
--------------------------------------------------------------------------------------------------------------------
14:23:45     2.50     -1.20    45.80    0.100    0.000    9.800    0.001    0.000    0.000    1234    567     890
```

## Data Fields

- **Euler Angles**: Roll, Pitch, Yaw (degrees)
  - Roll: Rotation around X-axis
  - Pitch: Rotation around Y-axis (used for gimbal stabilization)
  - Yaw: Rotation around Z-axis

- **Accelerometer**: X, Y, Z (m/s²)
  - Linear acceleration in each axis
  - At rest, Z should be ~9.8 m/s² (gravity)

- **Gyroscope**: X, Y, Z (rad/s)
  - Angular velocity in each axis
  - At rest, all should be near 0

- **Magnetometer**: X, Y, Z (microtesla, uT)
  - Magnetic field strength
  - Used for compass heading

- **Temperature**: Sensor temperature (if available)

## Finding the COM Port

### Windows

1. Open Device Manager
2. Go to "Ports (COM & LPT)"
3. Look for "Silicon Labs CP210x USB to UART Bridge (COMxx)"
4. Use the COM number (e.g., `COM7`)

### Linux

```bash
ls /dev/ttyUSB*  # Usually /dev/ttyUSB0
# or
ls /dev/ttyACM*  # Sometimes /dev/ttyACM0
```

## Troubleshooting

### "Serial port error"

- Make sure the ESP32 is connected via USB
- Check that the correct COM port is specified
- Ensure no other program is using the serial port (close Arduino IDE Serial Monitor, etc.)

### "No response from ESP32"

- Make sure the ESP32 is running the `pan_tilt_base_v0.9` firmware
- Check that the baud rate matches (default: 115200)
- Try resetting the ESP32

### "Received response but not IMU data"

- The ESP32 may be sending other data (feedback, debug messages)
- Try increasing the timeout or filtering responses

## Examples

```bash
# Single read
python imu_monitor.py --port COM7

# Continuous at 10 Hz
python imu_monitor.py --port COM7 --continuous --rate 10

# Table format at 2 Hz
python imu_monitor.py --port COM7 --table --rate 2

# Different baud rate
python imu_monitor.py --port COM7 --baud 9600
```

## Integration

This program uses the same JSON command interface as `pc_control.py`:
- Sends: `{"T": 126}` (CMD_GET_IMU_DATA)
- Receives: `{"T": 1002, "r": ..., "p": ..., "y": ..., ...}` (FEEDBACK_IMU_DATA)

You can also use `pc_control.py` to get IMU data:
```bash
python pc_control.py --port COM7 --json '{"T":126}' --read
```

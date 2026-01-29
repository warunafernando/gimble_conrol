# Pan-Tilt Board Test Utility - GUI Application

A comprehensive Windows GUI utility for testing all functions of the pan-tilt board.

## Features

### 1. Serial Connection
- COM port selection with auto-refresh
- Baud rate selection (115200, 230400, 460800, 921600)
- Connection status indicator
- Automatic reconnection handling

### 2. Pan/Tilt Control Tab
- **Absolute Position Control**: Sliders for pan (-180° to 180°) and tilt (-90° to 120°)
- **Single-Axis Control**: Control pan or tilt independently (auto-locks inactive axis)
- **Relative Move Control**: Move with specified speed
- **Axis Lock Control**: Lock/unlock pan and tilt axes individually
- **User Control**: Relative movement commands
- **Stop Button**: Emergency stop

### 3. IMU Data Tab
- **Euler Angles**: Roll, Pitch, Yaw (real-time display)
- **Accelerometer**: X, Y, Z axes in g
- **Gyroscope**: X, Y, Z axes in deg/s
- **Magnetometer**: X, Y, Z raw values
- **Temperature**: IMU temperature
- **Auto-update**: Optional 1 Hz automatic updates

### 4. Power Monitoring Tab
- **Bus Voltage**: Main power supply voltage
- **Load Voltage**: Voltage at load
- **Shunt Voltage**: Voltage across shunt resistor
- **Current**: Current consumption in mA
- **Power**: Power consumption in mW
- **Overflow Status**: INA219 overflow detection
- **Auto-update**: Optional 2 Hz automatic updates

### 5. Servo Feedback Tab
- **Pan Servo (ID 2)**:
  - Position (raw and degrees)
  - Speed
  - Load
  - Voltage
  - Temperature
  - Mode (Locked/Free)
- **Tilt Servo (ID 1)**: Same data as pan
- **Auto-update**: Optional 2 Hz automatic updates

### 6. System Control Tab
- **Heartbeat Control**: Set timeout and view status
- **Feedback Control**: Enable/disable periodic feedback, set interval
- **Quick Tests**:
  - Test All Sensors
  - Sweep Pan (full range test)
  - Sweep Tilt (full range test)
  - Figure-8 Pattern (coordination test)

### 7. Log/Console Tab
- Real-time message log (sent and received)
- Timestamped entries
- Auto-scroll option
- Clear log
- Save log to file

## Installation

### Requirements
```bash
pip install pyserial
```

Python 3.7+ with tkinter (usually included with Python on Windows)

### Run
```bash
python tools/pan_tilt_test_gui.py
```

Or create a Windows shortcut:
```bash
pythonw tools/pan_tilt_test_gui.py
```

## Usage

### Basic Workflow

1. **Connect**:
   - Select COM port (usually COM9 or COM6)
   - Select baud rate (921600 recommended)
   - Click "Connect"

2. **Test Functions**:
   - Use Pan/Tilt Control tab to move servos
   - Enable auto-update on IMU/Power/Servo tabs to see real-time data
   - Use System Control tab for configuration

3. **Monitor**:
   - Watch Log tab for all communication
   - Check data displays for sensor readings

### Quick Tests

- **Test All Sensors**: Requests IMU, INA219, and servo data
- **Sweep Pan**: Moves pan from -180° to 180° and back to center
- **Sweep Tilt**: Moves tilt from -90° to 120° and back to center
- **Figure-8 Pattern**: Executes a coordinated figure-8 motion pattern

## Protocol Commands Supported

| Command | Description | GUI Location |
|---------|-------------|--------------|
| T=133 | Pan/Tilt absolute | Pan/Tilt Control → Set Absolute |
| T=172 | Pan-only absolute | Pan/Tilt Control → Set Pan Only |
| T=173 | Tilt-only absolute | Pan/Tilt Control → Set Tilt Only |
| T=134 | Pan/Tilt move | Pan/Tilt Control → Move |
| T=135 | Stop | Pan/Tilt Control → Stop |
| T=141 | User control | Pan/Tilt Control → Send User Control |
| T=170 | Lock/unlock pan | Pan/Tilt Control → Lock Pan/Unlock Pan |
| T=171 | Lock/unlock tilt | Pan/Tilt Control → Lock Tilt/Unlock Tilt |
| T=126 | Get IMU data | IMU Data → Get IMU Data |
| T=160 | Get INA219 data | Power Monitoring → Get Power Data |
| T=131 | Enable/disable feedback | System Control → Enable/Disable Feedback |
| T=142 | Set feedback interval | System Control → Set Interval |
| T=136 | Set heartbeat timeout | System Control → Set Heartbeat |

## Feedback Messages Displayed

- **T=1002**: IMU data (roll, pitch, yaw, accel, gyro, mag, temp)
- **T=1010**: INA219 data (voltage, current, power, overflow)
- **T=1011**: Servo feedback (pan and tilt position, speed, load, voltage, temp, mode)
- **T=1012**: Heartbeat status (alive, timeout)

## Tips

1. **Start with low speeds** when testing movements
2. **Enable auto-update** on sensor tabs for real-time monitoring
3. **Watch the log** to see all communication
4. **Use quick tests** to verify basic functionality
5. **Check servo feedback** to verify movements are working
6. **Monitor power consumption** during movements

## Troubleshooting

**"Not Connected" error**:
- Check COM port is correct
- Ensure no other program is using the port
- Try different baud rate

**No data received**:
- Check board is powered on
- Verify firmware is running
- Check log for error messages

**Servos not moving**:
- Check servo feedback to see if they're locked
- Verify power supply is adequate (12V 5A)
- Check servo IDs are correct (Pan=2, Tilt=1)

## Screenshots

The GUI includes:
- Tabbed interface for organized testing
- Real-time data displays
- Visual feedback for all sensors
- Comprehensive logging
- Quick test functions

---

**Created**: 2026-01-26  
**Status**: Ready for testing  
**Platform**: Windows (tkinter-based, should work on Linux/Mac too)

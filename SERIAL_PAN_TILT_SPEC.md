# Pan/Tilt Serial Project (ST3215 + IMU + INA219)

High-level specification for a new ESP32 PlatformIO project derived from `pan_tilt_base_v0.9`.

## Goals

- Two serial bus servos (ST3215) for pan and tilt with full feedback.
- IMU + magnetometer data from QMI8658C and AK09918C.
- Power monitoring via INA219 (bus voltage, shunt voltage, current, power).
- Heartbeat safety to stop motion/torque on command loss.

## Non-Goals (Explicitly Excluded)

- Filesystem (LittleFS)
- WiFi (AP/STA/AP+STA)
- ESP-NOW
- UGV/motion control
- Robot arm control

## Hardware Overview

### ST3215 Serial Servos (Pan/Tilt)

- Bus type: TTL half-duplex serial (ST3215).
- Recommended bus settings: 1 Mbps, IDs in range 0–253.
- 360-degree absolute angle control with magnetic encoder, supports torque/feedback.
- Power: 6–12.6 V supply.  
Reference: [Waveshare ST3215 Servo](https://www.waveshare.com/wiki/ST3215_Servo)

**Expected feedback per servo (typical ST3215):**
- Position
- Speed
- Load
- Input voltage
- Temperature
- Mode/torque status

### IMU + Magnetometer

From `pan_tilt_base_v0.9`:
- **QMI8658C** (accelerometer + gyroscope)  
  I2C address: `0x6B` (`QMI8658_ADDR` in `QMI8658.h`)
- **AK09918C** (magnetometer)  
  I2C address: `0x0C` (`AK09918_I2C_ADDR` in `AK09918.h`)

IMU fusion in `IMU.cpp` uses AHRS (quaternion) and outputs roll/pitch/yaw with Kp/Ki gains.

### INA219 Power Monitor

From `battery_ctrl.h`:
- I2C address: `0x42`
- Shunt resistor: `0.01 Ω`
- Data captured: shunt voltage, bus voltage, load voltage, current, power, overflow

## Software Architecture (High-Level)

### Modules

- **SerialProtocol**: JSON parsing/serialization, newline-delimited framing.
- **ServoBus**: ST3215 bus initialization, pan/tilt motion control, feedback polling.
- **IMUService**: QMI8658C + AK09918C init, sensor reads, AHRS fusion.
- **PowerMonitor**: INA219 init, update readings, overflow detection.
- **HeartbeatSafety**: timeout watchdog, torque-off + stop on lost commands.

### Main Loop (Suggested)

1. `serialCtrl()` reads and parses JSON commands.
2. Update IMU data at fixed cadence (e.g., 50–100 Hz).
3. Update INA219 readings at lower cadence (e.g., 10–20 Hz).
4. Poll servo feedback (position, load, voltage, temp) at 20–50 Hz.
5. Heartbeat check: if timeout, stop motion and release torque.
6. Emit feedback messages when requested or at configured interval.

## Serial Protocol (High-Level)

### Message Framing

- JSON object per message (compact, no whitespace)
- Newline-terminated (`\n`)
- UTF-8 text (ASCII-safe fields)

### Command Set (Recommended)

Reuse existing command IDs where practical for compatibility with tools:

- **T=133**: Pan/Tilt absolute control  
  `{"T":133,"X":pan_deg,"Y":tilt_deg,"SPD":0,"ACC":0}`
- **T=172**: Pan-only absolute control (new)  
  `{"T":172,"X":pan_deg,"SPD":0,"ACC":0}`
- **T=173**: Tilt-only absolute control (new)  
  `{"T":173,"Y":tilt_deg,"SPD":0,"ACC":0}`
- **T=134**: Pan/Tilt move control  
  `{"T":134,"X":pan_deg,"Y":tilt_deg,"SX":300,"SY":300}`
- **T=174**: Pan-only move control (new)  
  `{"T":174,"X":pan_deg,"SX":300}`
- **T=175**: Tilt-only move control (new)  
  `{"T":175,"Y":tilt_deg,"SY":300}`
- **T=135**: Stop pan/tilt  
  `{"T":135}`
- **T=141**: User control (relative)  
  `{"T":141,"X":-1|0|1|2,"Y":-1|0|1|2,"SPD":300}`
- **T=170**: Lock/unlock pan axis (new)  
  `{"T":170,"cmd":0|1}`  
  `cmd=1` locks pan (torque on + hold), `cmd=0` unlocks pan (torque off)
- **T=171**: Lock/unlock tilt axis (new)  
  `{"T":171,"cmd":0|1}`  
  `cmd=1` locks tilt (torque on + hold), `cmd=0` unlocks tilt (torque off)
- **T=126**: Get IMU data (response T=1002)  
  `{"T":126}`
- **T=160**: Get INA219 data (proposed)  
  `{"T":160}`
- **T=131**: Enable/disable periodic feedback  
  `{"T":131,"cmd":0|1}`
- **T=142**: Feedback interval in ms  
  `{"T":142,"cmd":100}`
- **T=136**: Heartbeat timeout in ms  
  `{"T":136,"cmd":3000}`

If you prefer different `T` values, define them in a dedicated `json_cmd.h` for this project.

### Feedback Messages (Recommended)

#### IMU Data (Reuse existing)
**T=1002**
```json
{
  "T":1002,
  "r":roll,
  "p":pitch,
  "y":yaw,
  "ax":ax,"ay":ay,"az":az,
  "gx":gx,"gy":gy,"gz":gz,
  "mx":mx,"my":my,"mz":mz,
  "temp":temp
}
```

#### Power Monitor (Proposed)
**T=1010**
```json
{
  "T":1010,
  "bus_v":12.4,
  "shunt_mv":2.1,
  "load_v":12.402,
  "current_ma":120.5,
  "power_mw":1494.2,
  "overflow":0
}
```

#### Servo Feedback (Proposed)
**T=1011**
```json
{
  "T":1011,
  "pan":{"id":2,"pos":1234,"speed":50,"load":120,"v":12.3,"temp":32,"mode":0},
  "tilt":{"id":1,"pos":2047,"speed":20,"load":80,"v":12.3,"temp":30,"mode":0}
}
```

#### Heartbeat Status (Optional)
**T=1012**
```json
{"T":1012,"alive":1,"timeout_ms":3000}
```

## Heartbeat & Safety

**Behavior:**
- Every motion command refreshes `lastCmdRecvTime`.
- If `(millis() - lastCmdRecvTime) > heartbeatTimeout`:
  - Stop pan/tilt movement.
  - Disable servo torque (if safe to do so).
  - Optionally publish a heartbeat fault message.
 - When only one axis is commanded, the other axis should auto-lock (torque on + hold).
 - When an axis is idle (no target updates for a configured interval), it should auto-lock.

**Default timeout:** 3000 ms (changeable via `T=136`).

## Bus Speed & Messaging Optimization

### Serial Bus Speed (Host ↔ ESP32)

- Use **921600 baud** for the USB serial link where supported by the host/driver.
- Fall back to **460800** or **230400** if instability is observed.

### Servo Bus Speed (ESP32 ↔ ST3215)

- Use **1 Mbps** TTL bus (ST3215 default and maximum practical rate).
- Use half‑duplex UART with proper direction control and short cable lengths.

### Messaging Optimization

- Use compact JSON (no spaces) and short keys.
- Prefer **single‑axis commands** (`T=172/173/174/175`) to avoid sending unused fields.
- Send only on change where possible; avoid redundant periodic requests.
- Use configurable feedback intervals (`T=142`) and turn off periodic feedback when not needed (`T=131`).
- Keep IMU/INA219 feedback as **separate message types** to reduce payload size.
- Avoid floating precision bloat; round to needed decimals on sender.

## Build & Dependencies (PlatformIO)

Start from the `pan_tilt_base_v0.9` layout:
- `src_dir = .` (Arduino sketch + headers in root)
- `monitor_speed = 921600` (use highest stable rate)

Required libraries (minimum):
- `bblanchon/ArduinoJson`
- `wollewald/INA219_WE`
- `https://github.com/workloads/scservo.git` (ST series bus servo support)

IMU drivers are already present in the reference firmware:
- `QMI8658` and `AK09918` sources are local in the repo.

## Data Rates (Suggested)

- IMU update: 50–100 Hz
- Magnetometer: 100 Hz (AK09918 continuous)
- INA219: 10–20 Hz
- Servo feedback: 20–50 Hz
- Feedback stream: 10–20 Hz (configurable)

## Future Extensions (Optional)

- Magnetometer calibration workflow (offset capture/commands).
- Servo calibration/middle position config.
- Overcurrent protection using INA219.
- Logging to host (no local filesystem).

## References

- ST3215 servo specs and bus characteristics:  
  [https://www.waveshare.com/wiki/ST3215_Servo](https://www.waveshare.com/wiki/ST3215_Servo)

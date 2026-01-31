# ESP32 Pan-Tilt Program – Program Document

This document describes the **pan_tilt_serial_project** ESP32 firmware: what it does, how it is structured, and how to build and use it.

---

## 1. Purpose and Overview

| Item | Description |
|------|--------------|
| **Project** | `pan_tilt_serial_project` |
| **Board** | ESP32 Dev Module (espressif32 / Arduino) |
| **Role** | Serial-controlled pan-tilt gimbal: receives **binary** commands over USB serial, drives two ST3215 servos (pan/tilt), reads IMU and INA219 power monitor, and sends binary responses. |
| **Main file** | `pan_tilt_serial_project.ino` (~1013 lines) |
| **Build** | PlatformIO, `platformio.ini` → env `esp32dev` |

The program is **command-driven**: no WiFi, no filesystem. All host communication is over **USB serial** at **921600 baud** using a **binary protocol** (see [GIMBAL_PROTOCOL.md](GIMBAL_PROTOCOL.md)).

---

## 2. Hardware and Pinout

| Resource | Pin / Bus | Notes |
|----------|-----------|--------|
| **USB serial (host)** | UART0 (USB) | 921600 baud, binary protocol (STX/LEN/SEQ/TYPE/PAYLOAD/CRC8/ETX) |
| **Servo bus (ST3215)** | RX=18, TX=19 (`Serial1`) | 1 Mbps, half-duplex TTL; 2 servos: **ID 1 = Pan**, **ID 2 = Tilt** |
| **I2C (IMU + INA219)** | SDA=32, SCL=33 | Shared bus |
| **QMI8658C (accel/gyro)** | I2C 0x6B | 6-axis IMU |
| **AK09918C (magnetometer)** | I2C 0x0C | Magnetometer for AHRS |
| **INA219** | I2C 0x42 | Shunt 0.01 Ω; bus voltage, current, power |

Servos are powered externally (6–12.6 V); the ESP32 only drives the serial and I2C buses.

---

## 3. Source Files and Structure

| File | Role |
|------|------|
| `pan_tilt_serial_project.ino` | Config, globals, CRC8, frame send/parse, servo helpers, state machine, command switch (`processCommand`), `setup()` / `loop()` |
| `IMU.cpp` / `IMU.h` | IMU init, `imuDataGet()` (Euler angles, gyro/accel/mag), AHRS (quaternion, Kp/Ki) |
| `QMI8658.cpp` / `QMI8658.h` + `QMI8658reg.h` | QMI8658C driver (config, read acc/gyro) |
| `AK09918.cpp` / `AK09918.h` | AK09918C driver (continuous mode, read mag) |

**Libraries (PlatformIO):**

- **INA219_WE** (wollewald/INA219_WE) – power monitor
- **SCServo** (GitHub: workloads/scservo) – ST3215 servo protocol (`SMS_STS`)

---

## 4. Binary Protocol (Summary)

Full specification: **[GIMBAL_PROTOCOL.md](GIMBAL_PROTOCOL.md)**.

- **Frame format:** `[STX][LEN][SEQ][TYPE][PAYLOAD][CHECKSUM][ETX]`  
  STX=0x02, ETX=0x03. LEN = 4 + payload length. CRC8 over LEN..PAYLOAD (poly 0x07, init 0x00).
- **Host → ESP32:** Binary command frames (TYPE = 126, 133–136, 137, 139, 140, 160, 170–175, 200, 210–213, 501, 502, etc.).
- **ESP32 → Host:** Binary response frames: ACK_RECEIVED (1), ACK_EXECUTED (2, with 8-byte move feedback for move commands), NACK (3), IMU (1002), INA (1010), PING_RESP (2001), etc.
- **Move feedback:** For move commands (133, 134, 172–175), after executing the move the ESP32 reads position and torque from both servos and sends ACK_EXECUTED with 8 bytes: `pan_load(2), pan_pos(2), tilt_load(2), tilt_pos(2)`.

---

## 5. State Machine

| State | Enter | Allowed | Rejected |
|-------|-------|---------|----------|
| **IDLE** | Power-on, EXIT_CONFIG (140) | Move, config, query, state-change | — |
| **TRACKING** | ENTER_TRACKING (137) | Move (133, 134, 172–175), STOP (135), lock/unlock, IMU/INA, feedback, heartbeat | Config commands (200, 501, 210–213, 502) → NACK code 3 |
| **CONFIG** | ENTER_CONFIG (139) | Config commands, EXIT_CONFIG (140) | Move commands → NACK code 3 |

- **TRACKING** is a “fast path”: no `delay()` in move execution, no EPROM/ensurePositionMode per move (done once on entry). Position and torque are read immediately after each move and returned in ACK_EXECUTED.
- **Timer:** A hardware timer fires every 10 ms and sets `controlTick` (see §5b).

---

### 5a. State Machine (Detailed)

**States and variable**

- `enum GimbalState { IDLE, TRACKING, CONFIG };`
- Single global: `static GimbalState gimbalState = IDLE;`
- Updated only inside `processCommand()` when handling commands 137, 139, 140, or 135 (STOP in TRACKING → IDLE).

**Transitions (command → new state)**

| Command (TYPE) | Current state | New state | Side effect |
|----------------|---------------|-----------|-------------|
| **ENTER_TRACKING (137)** | any | **TRACKING** | `onEnterTracking()` then ACK_EXECUTED |
| **ENTER_CONFIG (139)** | any | **CONFIG** | ACK_EXECUTED only |
| **EXIT_CONFIG (140)** | any | **IDLE** | ACK_EXECUTED only |
| **PAN_TILT_STOP (135)** | TRACKING | **IDLE** | `stopPanTilt()`, ACK_EXECUTED |

No other commands change `gimbalState`. Power-on leaves the variable as **IDLE**.

**Guards (before the command switch)**

Guards run at the start of `processCommand()`. If a guard fails, the command is **rejected** with NACK code 3 (state rejected); `processCommand()` returns without changing state or executing the command.

1. **In TRACKING, config commands are rejected**
   - `isConfigType(type)` is true for: 200 (PING), 501 (SET_SERVO_ID), 210–213 (READ/WRITE byte/word), 502 (CALIBRATE).
   - If `gimbalState == TRACKING` and `isConfigType(type)`: `sendNack(seq, 3)` and return.

2. **In CONFIG, move commands are rejected**
   - `isMoveType(type)` is true for: 133, 134, 135, 172, 173, 174, 175 (all pan/tilt move and stop).
   - If `gimbalState == CONFIG` and `isMoveType(type)`: `sendNack(seq, 3)` and return.

All other commands (state-change 137/139/140, lock/unlock, IMU/INA, feedback, heartbeat, etc.) are allowed in every state (subject to their own payload checks).

**Per-state behavior**

- **IDLE**
  - Move commands use the “full” path: `setPanTiltAbs`, `setPanAbs`, `setTiltAbs`, etc. Each call does `ensurePositionMode`, `setServoTorqueLimitMax` (EPROM), `delay()`s, then `WritePosEx`/`SyncWritePosEx`. Slower, but correct for one-off moves and first-time setup.
  - Config commands (200, 501, 210–213, 502) run normally.
  - STOP (135) does not change state (already IDLE after stop).

- **TRACKING**
  - **Entry:** On ENTER_TRACKING (137), `onEnterTracking()` runs once: `ensurePositionMode` both servos, `setServoTorqueLimitMax` both (EPROM), `EnableTorque` both, set `panLocked = false`, `tiltLocked = false`. So EPROM and mode are done once; no per-move EPROM or `delay()` later.
  - **Move commands** use the “tracking” path: `setPanTiltAbsTracking`, `setPanAbsTracking`, `setTiltAbsTracking`, etc. These only: clamp degrees → convert to position → update `lastPanPosTracking` / `lastTiltPosTracking` → call `SyncWritePosEx` or `WritePosEx` (no `delay()`, no EPROM, no `ensurePositionMode`). Then `sendMoveFeedback(seq)` reads position and torque from both servos (no delay) and sends ACK_EXECUTED with 8-byte payload.
  - **STOP (135):** `stopPanTilt()`, then `gimbalState = IDLE`, then ACK_EXECUTED.
  - Config commands are **not** executed; they hit the guard and get NACK(3).

- **CONFIG**
  - Only config commands and EXIT_CONFIG (140) are useful. Move commands hit the guard and get NACK(3). Lock/unlock, IMU/INA, feedback, heartbeat still run if sent.

**Summary**

- One global state, three values: IDLE, TRACKING, CONFIG.
- Transitions only on 137 → TRACKING, 139 → CONFIG, 140 → IDLE, and 135 in TRACKING → IDLE.
- Guards: in TRACKING reject config types; in CONFIG reject move types.
- TRACKING entry does one-time servo/EPROM prep; move commands in TRACKING use a fast, no-delay path and return position/torque in ACK_EXECUTED.

---

### 5b. Timer Interrupt (Detailed)

**Purpose**

A hardware timer runs at a fixed period (10 ms) and sets a flag so the main loop can optionally re-apply the last pan/tilt target to the servo bus (e.g. to keep position in TRACKING). The interrupt does **not** drive servos or do I2C/Serial; it only sets the flag.

**Setup (in `setup()`)**

```c
controlTimer = timerBegin(0, 80, true);           // Timer 0, divider 80, count up
timerAttachInterrupt(controlTimer, &onControlTimer, true);  // edge true = alarm
timerAlarmWrite(controlTimer, 10000, true);        // alarm at 10000 counts, auto-reload
timerAlarmEnable(controlTimer);
```

- **Clock:** ESP32 timer clock is 80 MHz. Divider 80 → 1 MHz (1 µs per tick). Alarm at 10000 → **10 ms** period.
- **Auto-reload:** `true` means the timer reloads after alarm, so the interrupt fires every 10 ms.

**ISR (Interrupt Service Routine)**

```c
void IRAM_ATTR onControlTimer() {
  controlTick = true;
}
```

- **IRAM_ATTR:** Handler is placed in IRAM so it can run with cache disabled; avoids issues if flash is busy.
- **Only action:** Set `static volatile bool controlTick = true;`. No I2C, no Serial, no servo access. Keeps the ISR short and safe.

**Main loop and `controlTick`**

- `controlTick` is declared `volatile` so the compiler does not optimize away reads when the ISR sets it.
- In the current firmware, **`loop()` does not read `controlTick`**. The flag is set every 10 ms by the ISR but no code in `loop()` clears it or re-applies `lastPanPosTracking` / `lastTiltPosTracking` to the servos.
- **Possible use:** In `loop()`, if `controlTick` is true and `gimbalState == TRACKING`, clear `controlTick` and call `SyncWritePosEx` (or equivalent) with `lastPanPosTracking` and `lastTiltPosTracking` to re-apply the last commanded position. That would give a 10 ms “hold” cadence in TRACKING without doing it inside the ISR.

**Summary**

- **Timer:** ESP32 hardware timer 0, 80 MHz / 80 = 1 MHz, alarm 10000 → **10 ms** period, auto-reload.
- **ISR:** `onControlTimer()` in IRAM; only sets `controlTick = true`. No I2C, Serial, or servo.
- **Main loop:** Currently does not use `controlTick`; the interrupt is in place for optional future use (e.g. re-apply last position in TRACKING every 10 ms).

---

## 6. Program Flow

### 6.1 `setup()`

1. **Serial:** `Serial.begin(921600)` (host), `Serial1.begin(1000000)` (servos); `st.pSerial = &Serial1`.
2. **I2C:** `Wire.begin(SDA=32, SCL=33)`.
3. **INA219:** Init, ADC mode, gain, bus range, shunt 0.01 Ω.
4. **IMU:** `imuInit()` (QMI8658 + AK09918); first `updateImu()` and `updateIna219()`.
5. **Servos:** Position mode, torque limit max (EPROM), tilt angle limits (EPROM), torque on; both marked locked.
6. **Heartbeat:** `lastCmdRecvTime = millis()`.
7. **Timer:** `hw_timer_t` at 10 ms; ISR sets `controlTick` (no I2C/Serial/servo in ISR).

### 6.2 `loop()`

1. **serialCtrl()** – Non-blocking read into `frameBuf`. When a complete binary frame is present: validate CRC8, send ACK_RECEIVED (type 1), call `processCommand(seq, type, payload, len)`. For move commands: execute move, read position/torque (no delay in TRACKING), send ACK_EXECUTED (type 2) with 8-byte payload.
2. **IMU** – Every 10 ms: `updateImu()`.
3. **INA219** – Every 50 ms: `updateIna219()`.
4. **Heartbeat** – If `heartbeatTimeoutMs > 0` and no command for that long: `stopPanTilt()`, set `heartbeatStopActive`, send type 1012.
5. **Idle lock** – If an axis is unlocked and idle longer than `idleLockMs` (500 ms): re-enable torque to hold, set locked.
6. **Periodic feedback** – If `feedbackEnabled` and interval elapsed: `sendServoFeedbackBinary(0)`, `sendInaBinary(0)`.
7. **controlTick** – Currently unused in `loop()`; the 10 ms timer ISR sets it for optional future use (e.g. re-apply last pan/tilt in TRACKING). See §5b.

---

## 7. Command Handling (`processCommand`)

Incoming frames are parsed in `serialCtrl()`; valid frames trigger `processCommand(seq, type, payload, payloadLen)`.

- **State checks:** In TRACKING, config types (200, 501, 210–213, 502) → NACK(3). In CONFIG, move types → NACK(3).
- **State transitions:** TYPE 137 → TRACKING, 139 → CONFIG, 140 → IDLE.
- **Move commands (133, 134, 172–175):** Decode pan/tilt/speed/acc, call `setPanTiltAbs`, `setPanAbs`, `setTiltAbs`, etc. (or TRACKING variants), then `sendMoveFeedback(seq)` (8-byte ACK_EXECUTED).
- **Stop (135):** `stopPanTilt()`, ACK_EXECUTED.
- **Lock/Unlock (170, 171):** `lockAxis` / `unlockAxis` for pan or tilt.
- **Get IMU (126):** `sendImuBinary(seq)` → type 1002, 50 bytes.
- **Get INA (160):** `sendInaBinary(seq)` → type 1010, 21 bytes.
- **Feedback (131, 142), Heartbeat (136):** Set globals, ACK_EXECUTED.
- **Ping (200), Set ID (501), Read/Write byte/word (210–213), Calibrate (502):** Servo bus operations, type-specific responses (2001, 5002, 2101, etc.).
- **Get state (144):** `sendFrame(seq, FEEDBACK_STATE, &stateByte, 1)` with stateByte = 0 (IDLE), 1 (TRACKING), or 2 (CONFIG). The host can query whether the ESP32 is in tracking mode (or IDLE/CONFIG) at any time.
- **Unknown type:** NACK(2).

---

## 8. Servo Control

- **Ranges:** Pan ±180°, tilt −90° to 120° (configurable; tilt limits also set in servo EPROM).
- **Conversion:** `panDegToPos(deg)`, `tiltDegToPos(deg)` using `SERVO_CENTER` (2048) and `DEG_TO_POS` (4095/360). Tilt axis inverted.
- **Movement:** `setPanTiltAbs`, `setPanAbs`, `setTiltAbs`, `setPanTiltMove`, `setPanMove`, `setTiltMove` (and TRACKING variants). Before move: ensure position mode, torque limit max (EPROM if needed), enable torque, then `WritePosEx` / `SyncWritePosEx`.
- **Lock/Unlock:** Lock = read position, enable torque, optionally write position to hold. Unlock = torque off.
- **EPROM:** `setServoTorqueLimitMax`, `setTiltServoAngleLimits` (tilt min/max position for full range).

---

## 9. IMU and INA219

- **IMU:** `imuDataGet()` fills Euler angles (roll/pitch/yaw), gyro, accel, magnetometer, temp. AHRS in `IMU.cpp`. Response type 1002: 50 bytes (roll, pitch, yaw, ax–az, gx–gz, mx–mz, temp).
- **INA219:** `updateIna219()` reads shunt/bus voltage, current, power, overflow. Response type 1010: 21 bytes (bus_v, shunt_mv, load_v, current_ma, power_mw, overflow).

---

## 10. Build and Upload

### Build

```bash
cd pan_tilt_serial_project
pio run
```

Output: `.pio/build/esp32dev/firmware.bin` (and related artifacts).

### Upload

- **Monitor speed:** 921600 (in `platformio.ini`).
- **Upload:** Often requires **manual download mode** (hold BOOT, press RESET, then run upload).  
  See project docs (e.g. [FIRMWARE_UPLOAD_GUIDE.md](../FIRMWARE_UPLOAD_GUIDE.md), [WAVESHARE_FLASH_TOOL_GUIDE.md](../WAVESHARE_FLASH_TOOL_GUIDE.md)) for your board and tool.

```bash
pio run -t upload
```

---

## 11. Related Documents

| Document | Content |
|----------|---------|
| [GIMBAL_PROTOCOL.md](GIMBAL_PROTOCOL.md) | Binary protocol: frame format, command/response types, payload layouts |
| [ESP32_FIRMWARE_ANALYSIS.md](ESP32_FIRMWARE_ANALYSIS.md) | Detailed firmware analysis, legacy JSON reference, recommendations |
| [tools/INSTALL.md](tools/INSTALL.md) | Tooling and GUI setup |
| [tester/README.md](tester/README.md) | Auto-tester for serial commands |

---

## 12. Quick Reference

| Aspect | Detail |
|--------|--------|
| **Main role** | Binary serial pan-tilt controller with IMU and power telemetry |
| **Serial** | USB 921600, binary frames (STX/ETX, CRC8) |
| **Servos** | 2× ST3215 (ID 1=pan, ID 2=tilt), 1 Mbps on RX18/TX19 |
| **Sensors** | QMI8658C + AK09918C (IMU), INA219 (power) |
| **States** | IDLE, TRACKING (fast path), CONFIG |
| **Loop** | Serial → command; 10 ms IMU; 50 ms INA219; heartbeat; idle lock; optional periodic feedback |
| **Build** | PlatformIO `esp32dev`; upload via `pio run -t upload` (often with BOOT/RESET) |

This document describes the ESP32 program as of the current codebase. For protocol details, use [GIMBAL_PROTOCOL.md](GIMBAL_PROTOCOL.md).

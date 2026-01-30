# ESP32 Pan-Tilt Firmware – Full Analysis

Analysis of the **pan_tilt_serial_project** ESP32 firmware used for the Waveshare pan-tilt gimbal (ST3215 servos, IMU, INA219).

---

## 1. Overview & Purpose

| Item | Description |
|------|--------------|
| **Project** | `pan_tilt_serial_project` |
| **Board** | ESP32 Dev Module (espressif32 / Arduino) |
| **Role** | Serial-controlled pan-tilt controller: receives JSON over USB serial, drives two ST3215 servos (pan/tilt), reads IMU and power monitor, and sends feedback. |
| **Main file** | `pan_tilt_serial_project.ino` (~885 lines) |
| **Build** | PlatformIO, `platformio.ini` → env `esp32dev` |

The firmware is **command-driven**: no WiFi, no filesystem. As of the refactor, all host interaction is via **binary protocol** over USB serial (see GIMBAL_PROTOCOL.md); the previous JSON protocol is no longer used.

---

## 2. Hardware & Pinout

| Resource | Pin / Bus | Notes |
|----------|-----------|--------|
| **USB serial (host)** | UART0 (USB) | 921600 baud, binary protocol (STX/LEN/SEQ/TYPE/PAYLOAD/CRC8/ETX) per GIMBAL_PROTOCOL.md |
| **Servo bus (ST3215)** | RX=18, TX=19 (`Serial1`) | 1 Mbps, half-duplex TTL, 2 servos (ID 1=pan, ID 2=tilt) |
| **I2C (IMU + INA219)** | SDA=32, SCL=33 | Shared bus |
| **QMI8658C (accel/gyro)** | I2C 0x6B | 6-axis IMU |
| **AK09918C (magnetometer)** | I2C 0x0C | Magnetometer for AHRS |
| **INA219** | I2C 0x42 | Shunt 0.01 Ω, bus voltage, current, power |

Servos are powered externally (6–12.6 V); ESP32 only drives the serial bus and I2C.

---

## 3. Architecture & Modules

The program is **single-threaded**, all in one `.ino` plus local modules. No RTOS tasks.

### 3.1 Logical modules (in main .ino)

| Module | Responsibility |
|--------|----------------|
| **Serial protocol** | Read line-based JSON → `processCommand()`; build and print JSON feedback. |
| **Servo control** | Degree→position conversion, `setPan*`/`setTilt*`, lock/unlock, EPROM (torque limit, tilt angle limits). |
| **IMU** | `imuInit()` / `imuDataGet()` (QMI8658 + AK09918, AHRS in `IMU.cpp`). |
| **Power** | INA219 init and `updateIna219()` (shunt/bus voltage, current, power, overflow). |
| **Feedback** | `sendImuFeedback()`, `sendInaFeedback()`, `sendServoFeedback()` (T=1002, 1010, 1011). |
| **Safety** | Optional heartbeat timeout (stop + torque off), idle lock (re-enable torque to hold after idle). |

### 3.2 Source files

| File | Role |
|------|------|
| `pan_tilt_serial_project.ino` | Config, globals, servo helpers, command switch, `setup()`/`loop()`, all JSON handling. |
| `IMU.cpp` / `IMU.h` | IMU init, `imuDataGet()` (Euler angles, gyro/accel/mag), AHRS (quaternion, Kp/Ki). |
| `QMI8658.cpp/.h` + `QMI8658reg.h` | QMI8658C driver (config, read acc/gyro). |
| `AK09918.cpp/.h` | AK09918C driver (continuous mode, read mag). |

Libraries: **ArduinoJson**, **INA219_WE**, **SCServo** (ST3215: `SMS_STS`).

---

## 4. Binary Protocol (GIMBAL_PROTOCOL.md)

- **Framing**: binary frame = `[STX][LEN][SEQ][TYPE][PAYLOAD][CHECKSUM][ETX]`. STX=0x02, ETX=0x03. LEN = length of SEQ+TYPE+PAYLOAD (4+N). SEQ = 16-bit little-endian. TYPE = 16-bit little-endian. CHECKSUM = CRC8 over LEN..PAYLOAD (polynomial 0x07, init 0x00).
- **Direction**: host sends binary command frames; ESP32 replies with binary response frames (ACK_RECEIVED type 1, ACK_EXECUTED type 2 with optional 8-byte move feedback, NACK type 3, or type-specific responses 1002, 1010, 2001, etc.).
- **Move feedback**: For move commands (133, 134, 172–175), after executing the move the ESP32 reads position and torque from both servos on the servo bus and sends ACK_EXECUTED with 8-byte payload: pan_load(2), pan_pos(2), tilt_load(2), tilt_pos(2).
- **Half-duplex**: Applies only to the **servo bus** (Serial1). IMU is on I2C. Position and torque are read on the servo bus when a move is executed and returned in ACK_EXECUTED.

Single source of truth: [GIMBAL_PROTOCOL.md](GIMBAL_PROTOCOL.md).

---

## 4a. State Machine & Fast Path (refactor)

- **States**: IDLE, TRACKING, CONFIG.
- **IDLE**: Current-like behaviour; move and config commands allowed; optional EPROM on first move.
- **TRACKING**: Enter with T=137 (ENTER_TRACKING). Fast path only: move commands (133, 134, 172–175) and stop (135). No EPROM, no `ensurePositionMode`/`setServoTorqueLimitMax` per move (done once on TRACKING entry). Torque and position feedback in ACK_EXECUTED after each move. Config commands (200, 501, 210–213, 502) are **rejected** (NACK code 3).
- **CONFIG**: Enter with T=139 (ENTER_CONFIG). Servo config and EPROM only; move commands rejected or ignored. Exit with T=140 (EXIT_CONFIG) to IDLE.

**Timer interrupt**: `hw_timer_t` fires at 10 ms; ISR sets `controlTick`. In `loop()`, when `controlTick` is set and state is TRACKING, the last pan/tilt target is re-applied to the servo bus (no servo read in this path). IMU is on I2C and is read on request (T=126 → T=1002) or in the main loop.

**Fast path (TRACKING)**: `setPanTiltAbsTracking`, `setPanAbsTracking`, `setTiltAbsTracking`, etc. Only clamp degrees, convert to position, and `SyncWritePosEx`/`WritePosEx`; no `delay()`, no EPROM, no `ensurePositionMode` per move. After writing the move, position and torque are read from both servos back-to-back with **no delay** (`readServoFeedbackTracking`), then ACK_EXECUTED with 8-byte payload is sent to the host.

---

## 4b. Legacy Serial Protocol (reference only)

- **Framing**: one JSON object per message, newline (`\n`) terminated, UTF-8.
- **Direction**: host sends commands; ESP32 replies with JSON lines (feedback or responses). **Replaced by binary protocol above.**

### 4.1 Command set (incoming `T`)

| T | Name | Payload | Action |
|---|------|---------|--------|
| 133 | Pan/Tilt absolute | X, Y, SPD?, ACC? | Both servos to (X°, Y°). |
| 134 | Pan/Tilt move | X, Y, SX?, SY? | Both move with per-axis speed. |
| 135 | Stop | — | Torque off both. |
| 141 | User ctrl | X, Y, SPD? | Relative step (e.g. ±5°) or center. |
| 170 | Pan lock | cmd: 0/1 | Unlock/lock pan (torque off/on + hold). |
| 171 | Tilt lock | cmd: 0/1 | Unlock/lock tilt. |
| 172 | Pan only absolute | X, SPD?, ACC? | Pan only to X°. |
| 173 | Tilt only absolute | Y, SPD?, ACC? | Tilt only to Y°. |
| 174 | Pan only move | X, SX? | Pan move with speed. |
| 175 | Tilt only move | Y, SY? | Tilt move with speed. |
| 126 | Get IMU | — | Reply T=1002. |
| 160 | Get INA | — | Reply T=1010. |
| 131 | Feedback flow | cmd: 0/1 | Enable/disable periodic feedback. |
| 142 | Feedback interval | cmd: ms | Set feedback interval. |
| 136 | Heartbeat | cmd: ms | Set timeout (0=disable). |
| 200 | Ping servo | id | Reply T=2001 (result, mode, torque, position). |
| 501 | Set servo ID | from, to | Change servo ID; reply T=5002/5001/5003. |
| 210 | Read byte | id, addr | Reply T=2101. |
| 211 | Write byte | id, addr, value | Reply T=2111. |
| 212 | Read word | id, addr | Reply T=2121. |
| 213 | Write word | id, addr, value | Reply T=2131. |
| 502 | Calibrate | id | Set current position as middle; reply T=5021. |

### 4.2 Feedback (outgoing `T`)

| T | Content |
|---|--------|
| 1002 | IMU: r, p, y (roll/pitch/yaw), ax/ay/az, gx/gy/gz, mx/my/mz, temp. |
| 1010 | INA219: bus_v, shunt_mv, load_v, current_ma, power_mw, overflow. |
| 1011 | Servo: pan/tilt objects with id, pos, speed, load, v, temp, mode. |
| 1012 | Heartbeat status: alive, timeout_ms. |
| 2001 | Ping result. |
| 2101/2111/2121/2131/5021 | Register read/write and calibrate responses. |
| 5001/5002/5003 | Set-ID error/success/verification. |

---

## 5. Data Flow & Main Loop

### 5.1 `setup()`

1. Serial 921600, I2C (SDA=32, SCL=33).
2. `Serial1` 1 Mbps for servos; `st.pSerial = &Serial1`.
3. INA219 init (gain, range, shunt 0.01 Ω).
4. `imuInit()` (QMI8658 + AK09918, magnetometer continuous 100 Hz).
5. First `updateImu()` and `updateIna219()`.
6. Servos: position mode, torque limit max (EPROM), tilt angle limits (EPROM), torque on, both marked locked.
7. `lastCmdRecvTime = millis()` for heartbeat.
8. Timer: `hw_timer_t` at 10 ms period; ISR sets `controlTick` (no I2C/Serial/servo in ISR).

### 5.2 `loop()` (single thread)

1. **serialCtrl()** – non-blocking read into ring buffer; when a complete binary frame (STX…ETX) is present, validate CRC8, optionally send ACK_RECEIVED (type 1), decode TYPE and PAYLOAD, call `processCommand(seq, type, payload, len)`. For move commands: execute move, read position/torque from both servos (no delay in TRACKING), send ACK_EXECUTED (type 2) with 8-byte payload.
2. **IMU** – every `IMU_UPDATE_MS` (10 ms): `updateImu()`.
3. **INA219** – every `INA_UPDATE_MS` (50 ms): `updateIna219()`.
4. **Servo feedback** – not polled in loop; only when a move command is executed (feedback in ACK_EXECUTED) or when feedback is enabled and interval elapsed (periodic sendServoFeedbackBinary/sendInaBinary).
5. **Heartbeat** – if `heartbeatTimeoutMs > 0` and no command for that long: `stopPanTilt()`, set `heartbeatStopActive`, send T=1012 (binary).
6. **Idle lock** – if heartbeat not stopped and pan/tilt not locked: if idle longer than `idleLockMs` (500 ms), re-enable torque (hold), set locked.
7. **Periodic feedback** – if `feedbackEnabled` and interval elapsed: `sendServoFeedbackBinary(0)` and `sendInaBinary(0)`, update `lastFeedbackMs`.
8. **controlTick** – when timer ISR has set `controlTick`: clear flag; if state is TRACKING, re-apply last pan/tilt position to servo bus (optional redundant application).

So: **command handling and safety run in the same loop as IMU/INA219 updates**; move feedback is returned in ACK_EXECUTED; optional timer re-applies last position in TRACKING.

---

## 6. Servo Control Logic

### 6.1 Coordinate system

- **Pan**: ±180°, position = `SERVO_CENTER + deg * DEG_TO_POS` (2048, 4095/360).
- **Tilt**: −90° to 120°, position = `SERVO_CENTER - deg * DEG_TO_POS` (inverted).
- **ST3215**: 0–4095 (≈360°); center 2048.

### 6.2 Movement functions

- **setPanTiltAbs / setPanTiltMove** – both axes: ensure position mode, torque limit max, enable torque, `SyncWritePosEx` (or move variant).
- **setPanAbs / setTiltAbs** – single axis: same prep, `WritePosEx` for that servo only; other axis untouched.
- **setPanMove / setTiltMove** – single-axis move with speed, no acceleration.
- **stopPanTilt** – torque off both; both marked unlocked.

Before moves: **ensurePositionMode(id)** (mode 0), **setServoTorqueLimitMax(id)** (unlock EPROM, write 1000, lock EPROM), then enable torque and write position.

### 6.3 Lock / unlock

- **lockAxis(id, fb, lockedFlag)**: read feedback, enable torque, optionally write current position to hold (if pos in 1–4094). Sets `lockedFlag = true`.
- **unlockAxis(id, lockedFlag)**: torque off, `lockedFlag = false`.

Lock = “hold position”; unlock = relax (torque off).

### 6.4 EPROM and tilt limits

- **setServoTorqueLimitMax(id)**: unlock EPROM, write torque limit 1000 at `SMS_STS_TORQUE_LIMIT_L`, lock EPROM.
- **setTiltServoAngleLimits()**: tilt only; unlock EPROM, write min position 683 (120°), max 3072 (−90°), lock EPROM. Ensures full tilt range is allowed by servo firmware.

### 6.5 User control (T=141)

- X,Y in {-1, 0, 1, 2}: step pan/tilt by 5° or center (2,2 → 0°,0°). Targets clamped to pan ±180°, tilt −90°–120°. Then `setPanTiltAbs(panTargetDeg, tiltTargetDeg, …)`. If X==0 or Y==0, lock that axis.

---

## 7. IMU & INA219 Integration

### 7.1 IMU

- **imuDataGet()** fills: Euler angles (roll/pitch/yaw), gyro/accel (float), magnetometer (int16). AHRS in `IMU.cpp` (quaternion, Kp/Ki, `invSqrt`). Magnetometer offset applied (e.g. offset_x).
- **sendImuFeedback()** sends T=1002 with all fields (angles, accel, gyro, mag, temp).

### 7.2 INA219

- **updateIna219()** reads shunt/bus voltage, current, power, overflow.
- **sendInaFeedback()** sends T=1010 (bus_v, shunt_mv, load_v, current_ma, power_mw, overflow).

---

## 8. Safety

- **Heartbeat**: `DEFAULT_HEARTBEAT_MS = 0` (disabled). If set > 0, no command for that period → `stopPanTilt()` and T=1012.
- **Idle lock**: after `idleLockMs` (500 ms) without a pan/tilt command, axis is marked locked and torque is re-enabled to hold (no position write).
- **Servo feedback**: position written in lock only if read OK and in 1–4094 to avoid invalid hold.
- **JSON buffer**: serial buffer limited to 512 chars to avoid runaway memory.

---

## 9. Dependencies & Build

- **Platform**: espressif32, Arduino framework, board esp32dev.
- **Libraries**: ArduinoJson, INA219_WE, SCServo (GitHub: workloads/scservo — ST3215).
- **Build**: `pio run`; output under `.pio/build/esp32dev/` (e.g. `firmware.bin` at 0x10000).
- **Upload**: Documented to use `flash_esp32_cli.py` with manual download mode (BOOT+RESET); PlatformIO upload scripts may use Waveshare-specific flow.

---

## 10. Strengths

- Clear separation of commands (single switch in `processCommand()`).
- Pan/tilt ranges and tilt EPROM limits explicitly set; degree clamping and single-axis commands avoid accidental full-range moves.
- Lock/unlock and idle lock give predictable “hold” behavior.
- Optional heartbeat and configurable feedback interval.
- Register-level access (T=210–213, 502) for diagnostics and calibration.
- No WiFi/FS; simple serial-only surface and deterministic loop.

---

## 11. Weaknesses & Risks

1. **Blocking delays** – Multiple `delay(5)`/`delay(10)` in servo and EPROM paths; under heavy command traffic this can stall the loop and IMU/INA219 cadence.
2. **Single buffer** – One `serialBuffer` and one `jsonCmdReceive`; overlapping or malformed input can mix or drop data.
3. **No ACK/NACK** – Commands are executed but not explicitly acknowledged (only feedback T=1011 etc. when enabled); host cannot distinguish “received but failed” from “not received.”
4. **Servo feedback on demand** – Servo state is only read when feedback is enabled and timer fires or when locking; no continuous position check for “reached target.”
5. **IMU/INA219 in same loop** – Long servo/EPROM sequence can delay IMU update (e.g. 10 ms goal not met under load).
6. **Static JSON docs** – `StaticJsonDocument<256/512>`; very large or deeply nested input could overflow (deserialize only); 512 is reasonable but fixed.
7. **EulerAngles** – Defined in QMI8658.h but used by IMU; small coupling.

---

## 12. Recommendations

1. **Reduce blocking** – Replace fixed `delay()` in servo/EPROM paths with non-blocking state or short back-off so `loop()` returns quickly.
2. **Optional command ACK** – For critical commands (e.g. 133, 172, 173), send a short JSON ack (e.g. `{"T":133,"ack":1}`) so the host can retry or confirm.
3. **Position verification** – After a move command, optionally read servo position once and report “reached” or “position” in feedback (or in a dedicated response).
4. **Heartbeat default** – Consider leaving heartbeat disabled (0) for test GUI, but document and optionally enable (e.g. 3000 ms) for standalone/safety use.
5. **Document protocol** – Keep SERIAL_PROTOCOL_DOCUMENTATION.md (or equivalent) in sync with `processCommand()` and all T types.
6. **Unit tests** – Add tests for `panDegToPos`/`tiltDegToPos` and clamp ranges (e.g. in `test/` with PlatformIO).

---

## 13. Summary Table

| Aspect | Detail |
|--------|--------|
| **Main role** | JSON-over-serial pan-tilt controller with IMU and power telemetry |
| **Servos** | 2× ST3215 (pan=1, tilt=2), 1 Mbps UART, position mode, torque limit and tilt limits in EPROM |
| **Sensors** | QMI8658C + AK09918C (AHRS), INA219 (power) |
| **Protocol** | Newline-delimited JSON; T=133–175 motion, 126/160/131/142/136 config, 200/501/210–213/502 diag |
| **Loop** | Serial → command; 10 ms IMU; 50 ms INA219; optional heartbeat and idle lock; optional periodic feedback |
| **Safety** | Heartbeat (optional), idle lock, validated lock position |
| **Build** | PlatformIO esp32dev; upload via manual download or Waveshare script |

This document reflects the firmware as of the current codebase and can be updated when commands or behavior change.

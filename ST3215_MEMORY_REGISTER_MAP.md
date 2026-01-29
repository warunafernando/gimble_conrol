# ST3215 / SMS-STS Memory Register Map (EN)

Memory map derived from the SCServo library (SMS_STS) used for ST3215 serial bus servos. Addresses match the FIT SMS/STS protocol.

---

## Overview

| Region   | Address range (dec) | Access   | Description                    |
|----------|---------------------|----------|--------------------------------|
| EPROM    | 3–33                | R / R-W  | Non-volatile (ID, limits, mode) |
| SRAM     | 40–70                | R-W / R  | Runtime (goal, torque, feedback) |

**Note:** Some EPROM registers are read-only (e.g. model); others are read-write but may require EPROM unlock (write to LOCK) before writing.

---

## EPROM (Non-Volatile)

### Read-only

| Addr (dec) | Addr (hex) | Name          | Size | Access | Description |
|------------|------------|---------------|------|--------|-------------|
| 3          | 0x03       | MODEL_L       | 1    | R      | Model number low byte |
| 4          | 0x04       | MODEL_H       | 1    | R      | Model number high byte |

### Read-Write (may require EPROM unlock)

| Addr (dec) | Addr (hex) | Name               | Size | Access | Description |
|------------|------------|--------------------|------|--------|-------------|
| 5          | 0x05       | ID                | 1    | R-W    | Servo ID (1–254). Change may require unLockEprom → write → LockEprom. |
| 6          | 0x06       | BAUD_RATE         | 1    | R-W    | Serial baud rate setting |
| 9          | 0x09       | MIN_ANGLE_LIMIT_L | 1    | R-W    | Minimum angle limit low byte |
| 10         | 0x0A       | MIN_ANGLE_LIMIT_H | 1    | R-W    | Minimum angle limit high byte |
| 11         | 0x0B       | MAX_ANGLE_LIMIT_L | 1    | R-W    | Maximum angle limit low byte |
| 12         | 0x0C       | MAX_ANGLE_LIMIT_H | 1    | R-W    | Maximum angle limit high byte |
| **21**     | **0x15**   | **PID_P**         | **1**| **R-W**| **P gain (internal position PID). Write with EPROM unlock.** |
| **22**     | **0x16**   | **PID_D**         | **1**| **R-W**| **D gain (internal position PID). Write with EPROM unlock.** |
| **23**     | **0x17**   | **PID_I**         | **1**| **R-W**| **I gain (internal position PID). Write with EPROM unlock.** |
| 26         | 0x1A       | CW_DEAD           | 1    | R-W    | CW dead zone |
| 27         | 0x1B       | CCW_DEAD          | 1    | R-W    | CCW dead zone |
| 31         | 0x1F       | OFS_L             | 1    | R-W    | Position offset low byte (calibration) |
| 32         | 0x20       | OFS_H             | 1    | R-W    | Position offset high byte |
| 33         | 0x21       | MODE              | 1    | R-W    | 0 = position control, 1 = wheel (continuous) mode |

---

## SRAM (Runtime)

### Read-Write (control)

| Addr (dec) | Addr (hex) | Name              | Size | Access | Description |
|------------|------------|-------------------|------|--------|-------------|
| 40         | 0x28       | TORQUE_ENABLE     | 1    | R-W    | 0 = torque off, 1 = torque on. 128 = calibration (CalibrationOfs). |
| 41         | 0x29       | ACC               | 1    | R-W    | Acceleration (used with position/speed commands). |
| 42         | 0x2A       | GOAL_POSITION_L   | 1    | R-W    | Goal position low byte (0–4095, 2048 ≈ center). |
| 43         | 0x2B       | GOAL_POSITION_H   | 1    | R-W    | Goal position high byte. |
| 44         | 0x2C       | GOAL_TIME_L       | 1    | R-W    | Goal time low byte (optional). |
| 45         | 0x2D       | GOAL_TIME_H       | 1    | R-W    | Goal time high byte. |
| 46         | 0x2E       | GOAL_SPEED_L      | 1    | R-W    | Goal speed low byte (steps/s). |
| 47         | 0x2F       | GOAL_SPEED_H      | 1    | R-W    | Goal speed high byte. |
| 48         | 0x30       | TORQUE_LIMIT_L    | 1    | R-W    | Torque limit low byte (0–1000, 1000 = 100%). On some units, write may require EPROM unlock. |
| 49         | 0x31       | TORQUE_LIMIT_H    | 1    | R-W    | Torque limit high byte. |
| 55         | 0x37       | LOCK              | 1    | R-W    | 0 = EPROM unlock (allow write to ID, limits, etc.), 1 = EPROM lock. |

### Read-only (feedback)

| Addr (dec) | Addr (hex) | Name                 | Size | Access | Description |
|------------|------------|----------------------|------|--------|-------------|
| 56         | 0x38       | PRESENT_POSITION_L  | 1    | R      | Current position low byte (0–4095). |
| 57         | 0x39       | PRESENT_POSITION_H  | 1    | R      | Current position high byte. |
| 58         | 0x3A       | PRESENT_SPEED_L     | 1    | R      | Current speed low byte. |
| 59         | 0x3B       | PRESENT_SPEED_H     | 1    | R      | Current speed high byte. |
| 60         | 0x3C       | PRESENT_LOAD_L      | 1    | R      | Load (output %) low byte (0–1000). |
| 61         | 0x3D       | PRESENT_LOAD_H      | 1    | R      | Load high byte. |
| 62         | 0x3E       | PRESENT_VOLTAGE    | 1    | R      | Current voltage (unit per datasheet). |
| 63         | 0x3F       | PRESENT_TEMPERATURE| 1    | R      | Current temperature. |
| 66         | 0x42       | MOVING             | 1    | R      | Moving status. |
| 69         | 0x45       | PRESENT_CURRENT_L  | 1    | R      | Current (mA) low byte. |
| 70         | 0x46       | PRESENT_CURRENT_H  | 1    | R      | Current high byte. |

---

## Position and speed ranges (typical)

| Quantity   | Range    | Notes |
|-----------|----------|--------|
| Position  | 0–4095   | 2048 = center; 360° ≈ 4096 steps. |
| Speed     | 0–3400+  | Steps per second (direction in sign/bit per protocol). |
| Torque limit | 0–1000 | 1000 = 100% (max). |
| ACC       | 0–255    | Acceleration parameter for move commands. |

---

## Library constants (C/C++)

```c
// EPROM
#define SMS_STS_MODEL_L            3
#define SMS_STS_MODEL_H            4
#define SMS_STS_ID                 5
#define SMS_STS_BAUD_RATE           6
#define SMS_STS_MIN_ANGLE_LIMIT_L  9
#define SMS_STS_MIN_ANGLE_LIMIT_H  10
#define SMS_STS_MAX_ANGLE_LIMIT_L  11
#define SMS_STS_MAX_ANGLE_LIMIT_H  12
#define ST_PID_P_ADDR              21   // P gain (EPROM; unlock before write)
#define ST_PID_D_ADDR              22   // D gain (EPROM; unlock before write)
#define ST_PID_I_ADDR              23   // I gain (EPROM; unlock before write)
#define SMS_STS_CW_DEAD            26
#define SMS_STS_CCW_DEAD           27
#define SMS_STS_OFS_L              31
#define SMS_STS_OFS_H              32
#define SMS_STS_MODE               33

// SRAM (R-W)
#define SMS_STS_TORQUE_ENABLE      40
#define SMS_STS_ACC                41
#define SMS_STS_GOAL_POSITION_L    42
#define SMS_STS_GOAL_POSITION_H    43
#define SMS_STS_GOAL_TIME_L        44
#define SMS_STS_GOAL_TIME_H        45
#define SMS_STS_GOAL_SPEED_L       46
#define SMS_STS_GOAL_SPEED_H       47
#define SMS_STS_TORQUE_LIMIT_L     48
#define SMS_STS_TORQUE_LIMIT_H     49
#define SMS_STS_LOCK               55

// SRAM (R)
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L    58
#define SMS_STS_PRESENT_SPEED_H    59
#define SMS_STS_PRESENT_LOAD_L     60
#define SMS_STS_PRESENT_LOAD_H     61
#define SMS_STS_PRESENT_VOLTAGE    62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING             66
#define SMS_STS_PRESENT_CURRENT_L  69
#define SMS_STS_PRESENT_CURRENT_H  70
```

---

## Typical operations

1. **Read feedback:** Read from `SMS_STS_PRESENT_POSITION_L` (56) for 15 bytes (through `PRESENT_CURRENT_H`), or use library `FeedBack(ID)`.
2. **Position move:** Write ACC (41), then goal position (42–43) and goal speed (46–47), or use `WritePosEx(ID, Position, Speed, ACC)`.
3. **Torque limit:** Write 1000 to `SMS_STS_TORQUE_LIMIT_L` (48) and `TORQUE_LIMIT_H` (49). On some firmware/hardware, unlock EPROM first (`LOCK=0`), write, then `LOCK=1`.
4. **Torque on/off:** Write 1 or 0 to `SMS_STS_TORQUE_ENABLE` (40).
5. **Mode:** Write 0 (position) or 1 (wheel) to `SMS_STS_MODE` (33).

6. **PID (internal position controller):** P = addr 21, D = addr 22, I = addr 23 (1 byte each). Unlock EPROM (`LOCK=0`), write P/I/D, then lock (`LOCK=1`). Example values from pan_tilt_base_v0.9: RoArm default P = 16, default P = 32; I often 0.

---

*Source: SCServo library (SMS_STS) in pan_tilt_serial_project. For official ST3215 datasheet, see Waveshare ST3215 Servo wiki or manufacturer documentation.*

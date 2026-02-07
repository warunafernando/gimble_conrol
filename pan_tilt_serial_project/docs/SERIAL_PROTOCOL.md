# Gimbal Serial Protocol — Full Specification

This document describes the **binary serial protocol** used between a host (PC) and the ESP32 gimbal controller over UART. It was designed for reliable, framed messaging with a compact checksum and clear command/response types.

---

## 1. Physical and Link

- **Interface:** UART (e.g. USB‑serial adapter on ESP32 UART0).
- **Baud rate:** 921600 (recommended; same for normal operation and UART OTA).
- **Duplex:** Full duplex; host and ESP32 can send frames independently.
- **Byte order:** All multi-byte numeric fields are **little-endian** unless noted.

---

## 2. Frame Format

Every message is a single **frame**. No escaping of STX/ETX inside payloads in v1.

| Field    | Size    | Value / meaning |
|----------|---------|------------------|
| **STX**  | 1 byte  | `0x02` — start of frame |
| **LEN**  | 1 byte  | Length of the **body**: SEQ (2) + TYPE (2) + PAYLOAD = **4 + N**. Range 4–255, so payload length N ≤ 251. |
| **SEQ**  | 2 bytes | Sequence number 0–65535. Host increments per command; ESP32 echoes in responses. |
| **TYPE** | 2 bytes | Command or response type (see tables below). |
| **PAYLOAD** | N bytes | Type-specific data. N = LEN − 4. May be 0. |
| **CHECKSUM** | 1 byte | CRC8 over the **body** (LEN through last byte of PAYLOAD). |
| **ETX**  | 1 byte  | `0x03` — end of frame |

**Body** = bytes used for CRC8: `[LEN, SEQ_lo, SEQ_hi, TYPE_lo, TYPE_hi, PAYLOAD…]`. Body length = 1 + 2 + 2 + N = **5 + N** bytes.  
**Total frame length** = STX(1) + body(5+N) + CHECKSUM(1) + ETX(1) = **8 + N** bytes (minimum 8 when N=0). So **frame_size = 4 + LEN** (since LEN = 4+N).

### 2.1 CRC8 Algorithm

- **Polynomial:** 0x07 (x⁸ + x² + x + 1).
- **Initial value:** 0x00.
- **Final XOR:** none.
- **Input:** body bytes (LEN through last PAYLOAD byte).

C-style example:

```c
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (0x07 ^ (crc << 1)) : (crc << 1);
  }
  return crc & 0xFF;
}
```

Python (used in host `protocol.py`):

```python
def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (0x07 ^ (crc << 1)) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc
```

### 2.2 Parsing Rules

1. Locate STX (`0x02`). Discard bytes before it.
2. Read LEN (byte after STX). If LEN < 4 or LEN > 255, discard one byte and re-sync.
3. **Frame size** = **4 + LEN** bytes (STX + body + CHECKSUM + ETX). Body = bytes from LEN through PAYLOAD inclusive; body length = 1 + 2 + 2 + (LEN−4) = LEN+1. So total = 1 + (LEN+1) + 2 = 4+LEN.
4. Wait for 4+LEN bytes (including STX). Body = frame[1 : 2+LEN]. Validate frame[2+LEN] == CRC8(body) and frame[3+LEN] == ETX.
5. SEQ = body[1:3] (LE u16), TYPE = body[3:5] (LE u16), PAYLOAD = body[5 : 5+(LEN−4)].

---

## 3. Command Types (Host → ESP32)

TYPE is 2 bytes little-endian. Float = IEEE 754 single (4 bytes).

### 3.1 Motion and Control

| Dec | Hex   | Name           | Payload |
|-----|-------|----------------|---------|
| 126 | 0x007E | GET_IMU       | None (0 bytes). |
| 127 | 0x007F | GET_IMU2      | None. Second IMU (e.g. ICM42688). |
| 133 | 0x0085 | PAN_TILT_ABS  | pan(4 float), tilt(4 float), speed(2 u16), acc(2 u16) = 12 bytes. |
| 134 | 0x0086 | PAN_TILT_MOVE | pan(4), tilt(4), speed_pan(2), speed_tilt(2) = 12 bytes. |
| 135 | 0x0087 | PAN_TILT_STOP| None. |
| 141 | 0x008D | USER_CTRL     | x(1), y(1), speed(2) = 4 bytes. |
| 170 | 0x00AA | PAN_LOCK      | cmd(1): 0=unlock, 1=lock. |
| 171 | 0x00AB | TILT_LOCK     | cmd(1). |
| 172 | 0x00AC | PAN_ONLY_ABS  | pan(4 float), speed(2), acc(2) = 8 bytes. |
| 173 | 0x00AD | TILT_ONLY_ABS | tilt(4 float), speed(2), acc(2) = 8 bytes. |
| 174 | 0x00AE | PAN_ONLY_MOVE | pan(4), speed_pan(2) = 6 bytes. |
| 175 | 0x00AF | TILT_ONLY_MOVE| tilt(4), speed_tilt(2) = 6 bytes. |

### 3.2 Sensors and Power

| Dec | Hex   | Name    | Payload |
|-----|-------|---------|---------|
| 160 | 0x00A0 | GET_INA | None. |

### 3.3 Feedback and Heartbeat

| Dec | Hex   | Name               | Payload |
|-----|-------|--------------------|---------|
| 131 | 0x0083 | FEEDBACK_FLOW     | cmd(1): 0=disable, 1=enable. |
| 142 | 0x008E | FEEDBACK_INTERVAL | interval_ms(2). |
| 136 | 0x0088 | HEARTBEAT_SET     | timeout_ms(2); 0=disable. |

### 3.4 Servo Bus (SCS)

| Dec | Hex   | Name         | Payload |
|-----|-------|--------------|---------|
| 200 | 0x00C8 | PING_SERVO  | id(1). |
| 501 | 0x01F5 | SET_SERVO_ID| from_id(1), to_id(1). |
| 210 | 0x00D2 | READ_BYTE   | id(1), addr(1). |
| 211 | 0x00D3 | WRITE_BYTE  | id(1), addr(1), value(1). |
| 212 | 0x00D4 | READ_WORD   | id(1), addr(1). |
| 213 | 0x00D5 | WRITE_WORD  | id(1), addr(1), value(2). |
| 502 | 0x01F6 | CALIBRATE   | id(1). |

### 3.5 State and Mode

| Dec | Hex   | Name            | Payload |
|-----|-------|-----------------|---------|
| 137 | 0x0089 | ENTER_TRACKING | 0 bytes or interval_ms(2). |
| 139 | 0x008B | ENTER_CONFIG   | None. |
| 140 | 0x008C | EXIT_CONFIG    | None. |
| 144 | 0x0090 | GET_STATE      | None. Returns current state (IDLE/TRACKING/CONFIG). |

### 3.6 OTA (UART Firmware Update)

| Dec | Hex   | Name       | Payload |
|-----|-------|------------|---------|
| 600 | 0x0258 | OTA_START | total_size(4), hash_type(1), hash(N). hash_type: 0=none, 1=CRC32 (4 bytes), 2=SHA256 (32 bytes). |
| 601 | 0x0259 | OTA_CHUNK | offset(4), length(2), data(N). N ≤ 2048. |
| 602 | 0x025A | OTA_END   | None. Triggers verify and commit. |
| 603 | 0x025B | OTA_ABORT | None. Abort OTA. |

### 3.7 FW Info and Debug

| Dec | Hex   | Name       | Payload |
|-----|-------|------------|---------|
| 610 | 0x0262 | GET_FW_INFO | None. |
| 611 | 0x0263 | SWITCH_FW   | slot(1): 0=A (ota_0), 1=B (ota_1). Reboots. |
| 220 | 0x00DC | I2C_SCAN   | None. |

---

## 4. Response Types (ESP32 → Host)

### 4.1 Generic

| Dec | Hex   | Name          | Payload |
|-----|-------|---------------|---------|
| 1   | 0x0001 | ACK_RECEIVED | None. Sent after valid parse, before execution. |
| 2   | 0x0002 | ACK_EXECUTED | For move commands: pan_load(2), pan_pos(2), tilt_load(2), tilt_pos(2) = 8 bytes (int16). Else 0 bytes. |
| 3   | 0x0003 | NACK         | code(1), optional: msg_len(1), msg(msg_len). See NACK codes below. |

### 4.2 Sensors and Power

| Dec  | Hex    | Name | Payload layout |
|------|--------|------|----------------|
| 1002 | 0x03EA | IMU  | roll(4), pitch(4), yaw(4), ax(4), ay(4), az(4), gx(4), gy(4), gz(4), mx(2), my(2), mz(2), temp(4) = 50 bytes. All float except mx,my,mz = int16. |
| 1003 | 0x03F3 | IMU2 | ax(4), ay(4), az(4), gx(4), gy(4), gz(4), temp(4) = 28 bytes (e.g. ICM42688). |
| 1010 | 0x03F2 | INA  | bus_v(4), shunt_mv(4), load_v(4), current_ma(4), power_mw(4), overflow(1) = 21 bytes. Floats; overflow = 1 if overflow. |

### 4.3 State and Servo

| Dec  | Hex    | Name              | Payload |
|------|--------|-------------------|---------|
| 1011 | 0x03F3 | SERVO            | pan_pos(2), pan_load(2), tilt_pos(2), tilt_load(2) = 8 bytes. |
| 1012 | 0x03F4 | HEARTBEAT_STATUS | alive(1), timeout_ms(2). |
| 1013 | 0x03F5 | STATE            | state(1): 0=IDLE, 1=TRACKING, 2=CONFIG. |
| 2001 | 0x07D1 | PING_RESP        | id(1), responded(1), result(1), mode(1), torque_limit(2), torque_enable(1), position(2) = 9 bytes. |
| 5001 | 0x1389 | SET_ID_ERR       | error_code(1), optional msg. |
| 5002 | 0x138A | SET_ID_OK        | from(1), to(1). |
| 5003 | 0x138B | SET_ID_VERIFY    | id(1), verified(1). |
| 2101 | 0x0835 | READ_BYTE_RESP   | id(1), addr(1), value(1). |
| 2111 | 0x0847 | WRITE_BYTE_RESP  | id(1), addr(1), ok(1). |
| 2121 | 0x0849 | READ_WORD_RESP   | id(1), addr(1), value(2). |
| 2131 | 0x085B | WRITE_WORD_RESP  | id(1), addr(1), ok(1). |
| 5021 | 0x139D | CALIBRATE_RESP   | id(1), ok(1). |

### 4.4 OTA Responses

| Dec  | Hex    | Name            | Payload |
|------|--------|-----------------|---------|
| 2600 | 0x0A28 | OTA_STARTED     | inactive_slot(1), slot_size(4). Slot 0=A, 1=B. |
| 2601 | 0x0A29 | OTA_CHUNK       | bytes_written(4), progress_pct(1). Cumulative. |
| 2602 | 0x0A2A | OTA_DONE        | status(1). 0=OK (committed, rebooting). |
| 2603 | 0x0A2B | OTA_NACK        | error_code(1). See OTA error codes. |

### 4.5 FW Info Response

| Dec  | Hex    | Name    | Payload |
|------|--------|---------|---------|
| 2610 | 0x0A32 | FW_INFO | See below. |

**FW_INFO payload (70-byte format, current):**

| Offset | Size | Field       | Description |
|--------|------|-------------|-------------|
| 0      | 1    | active_slot | 0=A (ota_0), 1=B (ota_1). |
| 1      | 4    | serial      | uint32 LE, unique per unit (NVS). |
| 5      | 1    | model_id   | Product model (e.g. 99 for this gimbal). |
| 6      | 32   | version_a  | Null-padded ASCII; "---" if slot empty. |
| 38     | 32   | version_b  | Null-padded ASCII. |

**Legacy lengths:** 65 (no serial/model_id), 69 (serial, no model_id). Host should accept 65/69/70 and decode accordingly.

### 4.6 Debug

| Dec  | Hex    | Name | Payload |
|------|--------|------|---------|
| 2200 | 0x0898 | I2C_SCAN | count(1), addresses(count), optional WHO_AM_I data. |

---

## 5. NACK and OTA Error Codes

**NACK (type 3) codes:**

| Code | Meaning        |
|------|----------------|
| 1    | Checksum error |
| 2    | Unknown type   |
| 3    | State rejected |
| 4    | Execution failed |

**OTA_NACK (type 2603) codes:**

| Code | Name            | Description |
|------|-----------------|-------------|
| 1    | SIZE_MISMATCH   | Image size vs slot. |
| 2    | CHECKSUM_FAIL   | Hash verification failed. |
| 3    | FLASH_ERROR     | Write/erase error. |
| 4    | TIMEOUT         | No chunk in time. |
| 5    | ABORTED         | OTA_ABORT or internal abort. |

---

## 6. Flow Summary

1. Host sends one frame: STX, LEN, SEQ, TYPE, PAYLOAD, CHECKSUM, ETX.
2. ESP32 finds STX, validates LEN and CRC8, optionally sends ACK_RECEIVED (same SEQ), executes command, then sends ACK_EXECUTED (with 8-byte feedback for move commands) or a type-specific response (e.g. IMU, FW_INFO) or NACK.
3. For **move** commands (133, 134, 172, 173, 174, 175), ACK_EXECUTED carries pan_load, pan_pos, tilt_load, tilt_pos (int16, 8 bytes).
4. Unsolicited or async responses (e.g. IMU after GET_IMU) may use SEQ=0 or the request SEQ.

---

## 7. OTA Flow (UART Firmware Update)

1. Host sends **OTA_START** (total size, hash type, expected hash).
2. ESP32 replies **OTA_STARTED** (inactive slot, slot size).
3. Host sends **OTA_CHUNK** (offset, length, data) repeatedly. ESP32 replies **OTA_CHUNK** (bytes_written, progress_pct).
4. Host sends **OTA_END**. ESP32 verifies hash; on success commits and reboots; replies **OTA_DONE(0)** or **OTA_NACK**(error).

---

## 8. Reserved and Future Use

- **Escape:** If payloads ever need to contain 0x02 or 0x03, an escape (e.g. 0x10 0x02 → 0x02) can be defined later. Not required in v1.
- **LEN:** Values 0–3 reserved. Maximum payload 251 bytes.

---

## 9. Reference Implementation

- **Host (Python):** `backend/protocol.py` — `build_frame`, `parse_frame`, CRC8, and decode helpers for all response types.
- **Firmware:** `pan_tilt_serial_project.ino` — frame parser, command handlers, response builders.
- **Protocol summary:** `GIMBAL_PROTOCOL.md` in project root.

---

*Document version: 1.0 | Serial protocol for ESP32 gimbal controller*

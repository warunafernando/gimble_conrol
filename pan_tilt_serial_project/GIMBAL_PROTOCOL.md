# Gimbal Binary Protocol (Option A)

Binary framing and checksum for host–ESP32 gimbal communication over UART0. All multi-byte integers are **little-endian** unless noted.

---

## 1. Frame format

Every message (command or response) is a single frame:

| Field    | Size   | Value / meaning |
|----------|--------|------------------|
| STX      | 1 byte | `0x02` (start of frame) |
| LEN      | 1 byte | Length of SEQ + TYPE + PAYLOAD (2 + 2 + N = 4 + N). Range 4–255, so payload max 251 bytes. |
| SEQ      | 2 bytes| Sequence number (0–65535). Host increments per command; ESP32 echoes in response. |
| TYPE     | 2 bytes| Command or response type (see tables below). |
| PAYLOAD  | N bytes| Type-specific payload. N = LEN − 4. |
| CHECKSUM | 1 byte | CRC8 over all bytes from LEN through PAYLOAD (LEN, SEQ_lo, SEQ_hi, TYPE_lo, TYPE_hi, PAYLOAD…). |
| ETX      | 1 byte | `0x03` (end of frame) |

**Checksum scope**: CRC8 is computed over the byte sequence: `LEN`, `SEQ[0]`, `SEQ[1]`, `TYPE[0]`, `TYPE[1]`, then all PAYLOAD bytes. STX and ETX are **not** included in the checksum.

**CRC8**: Use a standard CRC-8 (e.g. polynomial 0x07, init 0x00, no final XOR). Same algorithm on host and ESP32 so that received frames can be validated and sent frames can be built with a valid checksum.

**Minimum frame size**: 8 bytes (STX + LEN + SEQ + TYPE + CHECKSUM + ETX with LEN=4, no payload).

---

## 2. Command types (host → ESP32)

TYPE is 2 bytes little-endian. Commands from the current program:

| TYPE (hex) | Dec | Name | Payload layout |
|------------|-----|------|-----------------|
| 0x007E     | 126 | GET_IMU | None (LEN=4). |
| 0x0085     | 133 | PAN_TILT_ABS | X(4 float), Y(4 float), SPD(2), ACC(2) = 12 bytes. |
| 0x0086     | 134 | PAN_TILT_MOVE | X(4), Y(4), SX(2), SY(2) = 12 bytes. |
| 0x0087     | 135 | PAN_TILT_STOP | None. |
| 0x008D     | 141 | USER_CTRL | X(1), Y(1), SPD(2) = 4 bytes. |
| 0x00AA     | 170 | PAN_LOCK | cmd(1) = 0 unlock, 1 lock. |
| 0x00AB     | 171 | TILT_LOCK | cmd(1). |
| 0x00AC     | 172 | PAN_ONLY_ABS | X(4 float), SPD(2), ACC(2) = 8 bytes. |
| 0x00AD     | 173 | TILT_ONLY_ABS | Y(4 float), SPD(2), ACC(2) = 8 bytes. |
| 0x00AE     | 174 | PAN_ONLY_MOVE | X(4), SX(2) = 6 bytes. |
| 0x00AF     | 175 | TILT_ONLY_MOVE | Y(4), SY(2) = 6 bytes. |
| 0x00A0     | 160 | GET_INA | None. |
| 0x0083     | 131 | FEEDBACK_FLOW | cmd(1) = 0 disable, 1 enable. |
| 0x008E     | 142 | FEEDBACK_INTERVAL | cmd(2) = interval ms. |
| 0x0088     | 136 | HEARTBEAT_SET | cmd(2) = timeout ms, 0 = disable. |
| 0x00C8     | 200 | PING_SERVO | id(1). |
| 0x01F5     | 501 | SET_SERVO_ID | from(1), to(1) = 2 bytes. |
| 0x00D2     | 210 | READ_BYTE | id(1), addr(1) = 2 bytes. |
| 0x00D3     | 211 | WRITE_BYTE | id(1), addr(1), value(1) = 3 bytes. |
| 0x00D4     | 212 | READ_WORD | id(1), addr(1) = 2 bytes. |
| 0x00D5     | 213 | WRITE_WORD | id(1), addr(1), value(2) = 4 bytes. |
| 0x01F6     | 502 | CALIBRATE | id(1). |
| 0x0089     | 137 | ENTER_TRACKING | optional: interval(2) ms. 0 bytes or 2 bytes. |
| 0x008B     | 139 | ENTER_CONFIG | None. |
| 0x008C     | 140 | EXIT_CONFIG | None. |

Float: IEEE 754 single (4 bytes). All multi-byte fields little-endian.

---

## 3. Response types (ESP32 → host)

| TYPE (hex) | Dec | Name | Payload layout |
|------------|-----|------|----------------|
| 0x0001     | 1   | ACK_RECEIVED | None. Sent after valid frame parse, before execution. |
| 0x0002     | 2   | ACK_EXECUTED | For **move** commands (133,134,172,173,174,175): feedback = **position and torque** for both axes. Payload: pan_load(2), pan_pos(2), tilt_load(2), tilt_pos(2) = 8 bytes. For other commands: 0 bytes. |
| 0x0003     | 3   | NACK | code(1), then optional msg: mlen(1), msg(mlen). code: 1=checksum err, 2=unknown type, 3=state rejected, 4=execution failed. |
| 0x03EA     | 1002 | IMU | roll(4), pitch(4), yaw(4), ax(4), ay(4), az(4), gx(4), gy(4), gz(4), mx(2), my(2), mz(2), temp(4) = 50 bytes. |
| 0x03F2     | 1010 | INA | bus_v(4), shunt_mv(4), load_v(4), current_ma(4), power_mw(4), overflow(1) = 21 bytes. |
| 0x03F3     | 1011 | SERVO | pan: pos(2), load(2), tilt: pos(2), load(2) = 8 bytes. (Used if periodic servo feedback is ever added; move commands use ACK_EXECUTED with torque instead.) |
| 0x03F4     | 1012 | HEARTBEAT_STATUS | alive(1), timeout_ms(2) = 3 bytes. |
| 0x07D1     | 2001 | PING_RESP | id(1), responded(1), result(1), mode(1), torque_limit(2), torque_enable(1), position(2) = 9 bytes. |
| 0x138A     | 5002 | SET_ID_OK | from(1), to(1) = 2 bytes. |
| 0x138B     | 5003 | SET_ID_VERIFY | id(1), verified(1) = 2 bytes. |
| 0x1389     | 5001 | SET_ID_ERR | error_code(1), then optional msg. |
| 0x0835     | 2101 | READ_BYTE_RESP | id(1), addr(1), value(1) = 3 bytes. |
| 0x0847     | 2111 | WRITE_BYTE_RESP | id(1), addr(1), ok(1) = 3 bytes. |
| 0x0849     | 2121 | READ_WORD_RESP | id(1), addr(1), value(2) = 4 bytes. |
| 0x085B     | 2131 | WRITE_WORD_RESP | id(1), addr(1), ok(1) = 3 bytes. |
| 0x139D     | 5021 | CALIBRATE_RESP | id(1), ok(1) = 2 bytes. |

Unsolicited or async responses (e.g. IMU on request): SEQ can be 0 or the same as the request if the response is tied to a command.

---

## 4. Flow

1. **Host** sends one frame: STX, LEN, SEQ, TYPE, PAYLOAD, CHECKSUM, ETX.
2. **ESP32** receives, finds STX/ETX, checks LEN consistency, computes CRC8 over (LEN..PAYLOAD), compares CHECKSUM. If invalid → optionally send NACK(code=1). If valid → optionally send ACK_RECEIVED (same SEQ), execute command, then send ACK_EXECUTED (with torque for move commands) or NACK(code 2–4) or a type-specific response (e.g. 2001, 2101).
3. For **move commands** (133, 134, 172, 173, 174, 175): ESP32 writes position to servo bus, reads **position and torque** from both servos, then sends ACK_EXECUTED with payload `pan_load, pan_pos, tilt_load, tilt_pos` (8 bytes). Feedback when move is executed = **position and torque** for both pan and tilt.

---

## 5. CRC8 algorithm

Use CRC-8 with polynomial 0x07 (x^8 + x^2 + x + 1), initial value 0x00, no final XOR. Input: bytes from LEN through last byte of PAYLOAD. Output: 1 byte.

Example (C-style) for computing checksum when building or validating a frame:

```c
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (0x07 ^ (crc << 1)) : (crc << 1);
  }
  return crc;
}
```

Checksum is computed over: `data = [LEN, SEQ_lo, SEQ_hi, TYPE_lo, TYPE_hi, PAYLOAD...]`, `len = 1 + 2 + 2 + payload_len = 5 + N`.

---

## 6. Escape / reserved bytes

If binary payload or future extension could contain 0x02 or 0x03, the protocol can later define an escape (e.g. 0x10 0x02 → 0x02, 0x10 0x03 → 0x03). For current payloads (floats, IDs, small integers) STX/ETX do not appear in payload; escape is not required in v1.

---

## 7. Summary

- **Option A**: Binary frame = `[STX][LEN][SEQ][TYPE][PAYLOAD][CHECKSUM][ETX]`.
- **Checksum**: CRC8 over LEN through PAYLOAD (single byte, fast).
- **SEQ**: 16-bit, echoed in responses for matching.
- **ACK_RECEIVED** (type 1): optional, after valid parse.
- **ACK_EXECUTED** (type 2): after execution; for move commands feedback = **position and torque** for both axes, payload = 8 bytes (pan_load, pan_pos, tilt_load, tilt_pos).
- **NACK** (type 3): code + optional message.
- Half-duplex applies only to the **servo bus** (Serial1). **Position and torque** are read on that bus when a move is executed and returned in ACK_EXECUTED (8-byte payload). IMU is on I2C.

This document defines the protocol only. Implementation (parser/serializer on ESP32 and host) follows this spec.

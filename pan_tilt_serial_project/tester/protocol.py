# Binary gimbal protocol per GIMBAL_PROTOCOL.md (copy for self-contained tester)
# Frame: STX LEN SEQ TYPE PAYLOAD CHECKSUM ETX
# CRC8 over LEN..PAYLOAD (poly 0x07, init 0x00)

import struct
from typing import Optional, Tuple

STX = 0x02
ETX = 0x03
MAX_PAYLOAD = 251

# Response types
RSP_ACK_RECEIVED = 1
RSP_ACK_EXECUTED = 2
RSP_NACK = 3

# Command types (host -> ESP32)
CMD_GET_IMU = 126
CMD_PAN_TILT_ABS = 133
CMD_PAN_TILT_MOVE = 134
CMD_PAN_TILT_STOP = 135
CMD_USER_CTRL = 141
CMD_PAN_LOCK = 170
CMD_TILT_LOCK = 171
CMD_PAN_ONLY_ABS = 172
CMD_TILT_ONLY_ABS = 173
CMD_PAN_ONLY_MOVE = 174
CMD_TILT_ONLY_MOVE = 175
CMD_GET_INA = 160
CMD_FEEDBACK_FLOW = 131
CMD_FEEDBACK_INTERVAL = 142
CMD_HEARTBEAT_SET = 136
CMD_PING_SERVO = 200
CMD_SET_SERVO_ID = 501
CMD_READ_BYTE = 210
CMD_WRITE_BYTE = 211
CMD_READ_WORD = 212
CMD_WRITE_WORD = 213
CMD_CALIBRATE = 502
CMD_ENTER_TRACKING = 137
CMD_ENTER_CONFIG = 139
CMD_EXIT_CONFIG = 140


def crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (0x07 ^ (crc << 1)) if (crc & 0x80) else (crc << 1)
        crc &= 0xFF
    return crc


def build_frame(seq: int, type_id: int, payload: Optional[bytes] = None) -> bytes:
    payload = payload or b""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too long")
    length = 4 + len(payload)
    body = struct.pack("<BHH", length, seq & 0xFFFF, type_id & 0xFFFF) + payload
    checksum = crc8(body)
    return bytes([STX]) + body + bytes([checksum, ETX])


def parse_frame(data: bytes) -> Optional[Tuple[int, int, bytes]]:
    if len(data) < 8 or data[0] != STX:
        return None
    length = data[1]
    if length < 4 or length > MAX_PAYLOAD:
        return None
    frame_size = 4 + length
    if len(data) < frame_size:
        return None
    if data[frame_size - 1] != ETX:
        return None
    body = data[1 : frame_size - 2]
    if crc8(body) != data[frame_size - 2]:
        return None
    seq = struct.unpack_from("<H", body, 1)[0]
    type_id = struct.unpack_from("<H", body, 3)[0]
    payload = body[5:5 + (length - 4)] if length > 4 else b""
    return (seq, type_id, payload)


def decode_move_feedback(payload: bytes) -> Optional[dict]:
    if len(payload) < 8:
        return None
    pan_load = struct.unpack_from("<h", payload, 0)[0]
    pan_pos = struct.unpack_from("<h", payload, 2)[0]
    tilt_load = struct.unpack_from("<h", payload, 4)[0]
    tilt_pos = struct.unpack_from("<h", payload, 6)[0]
    return {"pan_load": pan_load, "pan_pos": pan_pos, "tilt_load": tilt_load, "tilt_pos": tilt_pos}


def decode_imu(payload: bytes) -> Optional[dict]:
    if len(payload) < 50:
        return None
    return {
        "roll": struct.unpack_from("<f", payload, 0)[0],
        "pitch": struct.unpack_from("<f", payload, 4)[0],
        "yaw": struct.unpack_from("<f", payload, 8)[0],
        "ax": struct.unpack_from("<f", payload, 12)[0],
        "ay": struct.unpack_from("<f", payload, 16)[0],
        "az": struct.unpack_from("<f", payload, 20)[0],
        "gx": struct.unpack_from("<f", payload, 24)[0],
        "gy": struct.unpack_from("<f", payload, 28)[0],
        "gz": struct.unpack_from("<f", payload, 32)[0],
        "mx": struct.unpack_from("<h", payload, 36)[0],
        "my": struct.unpack_from("<h", payload, 38)[0],
        "mz": struct.unpack_from("<h", payload, 40)[0],
        "temp": struct.unpack_from("<f", payload, 46)[0],
    }

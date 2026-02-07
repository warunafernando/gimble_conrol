# Binary gimbal protocol per GIMBAL_PROTOCOL.md
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
CMD_GET_IMU2 = 127  # ICM42688 second IMU
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
CMD_GET_STATE = 144

# OTA Commands
CMD_OTA_START = 600
CMD_OTA_CHUNK = 601
CMD_OTA_END = 602
CMD_OTA_ABORT = 603

# FW Info Commands
CMD_GET_FW_INFO = 610
CMD_SWITCH_FW = 611

# Debug Commands
CMD_I2C_SCAN = 220
RSP_I2C_SCAN = 2200

# OTA Responses
RSP_OTA_STARTED = 2600
RSP_OTA_CHUNK = 2601
RSP_OTA_DONE = 2602
RSP_OTA_NACK = 2603
RSP_FW_INFO = 2610

FW_VERSION_LEN = 32

# OTA hash types
OTA_HASH_NONE = 0
OTA_HASH_CRC32 = 1
OTA_HASH_SHA256 = 2

# OTA error codes
OTA_ERR_SIZE_MISMATCH = 1
OTA_ERR_CHECKSUM_FAIL = 2
OTA_ERR_FLASH_ERROR = 3
OTA_ERR_TIMEOUT = 4
OTA_ERR_ABORTED = 5


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


def decode_ina(payload: bytes) -> Optional[dict]:
    if len(payload) < 21:
        return None
    return {
        "bus_v": struct.unpack_from("<f", payload, 0)[0],
        "shunt_mv": struct.unpack_from("<f", payload, 4)[0],
        "load_v": struct.unpack_from("<f", payload, 8)[0],
        "current_ma": struct.unpack_from("<f", payload, 12)[0],
        "power_mw": struct.unpack_from("<f", payload, 16)[0],
        "overflow": payload[20] if len(payload) > 20 else 0,
    }


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


def decode_imu2(payload: bytes) -> Optional[dict]:
    """Decode IMU2 (ICM42688) response: 28 bytes."""
    if len(payload) < 28:
        return None
    return {
        "ax": struct.unpack_from("<f", payload, 0)[0],
        "ay": struct.unpack_from("<f", payload, 4)[0],
        "az": struct.unpack_from("<f", payload, 8)[0],
        "gx": struct.unpack_from("<f", payload, 12)[0],
        "gy": struct.unpack_from("<f", payload, 16)[0],
        "gz": struct.unpack_from("<f", payload, 20)[0],
        "temp": struct.unpack_from("<f", payload, 24)[0],
    }


# Human-readable log helpers
CMD_NAMES = {
    CMD_GET_IMU: "GET_IMU",
    CMD_GET_IMU2: "GET_IMU2",
    CMD_PAN_TILT_ABS: "PAN_TILT_ABS",
    CMD_PAN_TILT_MOVE: "PAN_TILT_MOVE",
    CMD_PAN_TILT_STOP: "PAN_TILT_STOP",
    CMD_USER_CTRL: "USER_CTRL",
    CMD_PAN_LOCK: "PAN_LOCK",
    CMD_TILT_LOCK: "TILT_LOCK",
    CMD_PAN_ONLY_ABS: "PAN_ONLY_ABS",
    CMD_TILT_ONLY_ABS: "TILT_ONLY_ABS",
    CMD_PAN_ONLY_MOVE: "PAN_ONLY_MOVE",
    CMD_TILT_ONLY_MOVE: "TILT_ONLY_MOVE",
    CMD_GET_INA: "GET_INA",
    CMD_FEEDBACK_FLOW: "FEEDBACK_FLOW",
    CMD_FEEDBACK_INTERVAL: "FEEDBACK_INTERVAL",
    CMD_HEARTBEAT_SET: "HEARTBEAT_SET",
    CMD_PING_SERVO: "PING_SERVO",
    CMD_SET_SERVO_ID: "SET_SERVO_ID",
    CMD_READ_BYTE: "READ_BYTE",
    CMD_WRITE_BYTE: "WRITE_BYTE",
    CMD_READ_WORD: "READ_WORD",
    CMD_WRITE_WORD: "WRITE_WORD",
    CMD_CALIBRATE: "CALIBRATE",
    CMD_ENTER_TRACKING: "ENTER_TRACKING",
    CMD_ENTER_CONFIG: "ENTER_CONFIG",
    CMD_EXIT_CONFIG: "EXIT_CONFIG",
    CMD_GET_STATE: "GET_STATE",
    CMD_OTA_START: "OTA_START",
    CMD_OTA_CHUNK: "OTA_CHUNK",
    CMD_OTA_END: "OTA_END",
    CMD_OTA_ABORT: "OTA_ABORT",
    CMD_I2C_SCAN: "I2C_SCAN",
    CMD_GET_FW_INFO: "GET_FW_INFO",
    CMD_SWITCH_FW: "SWITCH_FW",
}
RSP_NAMES = {
    RSP_ACK_RECEIVED: "ACK_RECEIVED",
    RSP_ACK_EXECUTED: "ACK_EXECUTED",
    RSP_NACK: "NACK",
    1002: "IMU",
    1003: "IMU2",
    1010: "INA",
    1011: "SERVO",
    1012: "HEARTBEAT_STATUS",
    1013: "STATE",
    2001: "PING_RESP",
    5002: "SET_ID_OK",
    5003: "SET_ID_VERIFY",
    5001: "SET_ID_ERR",
    2101: "READ_BYTE_RESP",
    2111: "WRITE_BYTE_RESP",
    2121: "READ_WORD_RESP",
    2131: "WRITE_WORD_RESP",
    5021: "CALIBRATE_RESP",
    RSP_OTA_STARTED: "OTA_STARTED",
    RSP_OTA_CHUNK: "OTA_CHUNK_ACK",
    RSP_OTA_DONE: "OTA_DONE",
    RSP_OTA_NACK: "OTA_NACK",
    RSP_FW_INFO: "FW_INFO",
    RSP_I2C_SCAN: "I2C_SCAN_RESULT",
}


def format_tx_for_log(seq: int, type_id: int, payload: bytes) -> str:
    """Human-readable TX line for debug log."""
    name = CMD_NAMES.get(type_id, f"CMD_{type_id}")
    try:
        if type_id == CMD_PAN_TILT_ABS and len(payload) >= 12:
            pan, tilt = struct.unpack_from("<ff", payload, 0)
            spd, acc = struct.unpack_from("<HH", payload, 8)
            return f"TX {name} seq={seq} pan={pan:.1f} tilt={tilt:.1f} spd={spd} acc={acc}"
        if type_id == CMD_PAN_TILT_MOVE and len(payload) >= 12:
            pan, tilt = struct.unpack_from("<ff", payload, 0)
            sx, sy = struct.unpack_from("<HH", payload, 8)
            return f"TX {name} seq={seq} pan={pan:.1f} tilt={tilt:.1f} sx={sx} sy={sy}"
        if type_id == CMD_PAN_TILT_STOP:
            return f"TX {name} seq={seq}"
        if type_id == CMD_USER_CTRL and len(payload) >= 4:
            x, y = struct.unpack_from("<bb", payload, 0)[0], struct.unpack_from("<bb", payload, 1)[0]
            spd = struct.unpack_from("<H", payload, 2)[0]
            return f"TX {name} seq={seq} x={x} y={y} spd={spd}"
        if type_id in (CMD_PAN_LOCK, CMD_TILT_LOCK) and len(payload) >= 1:
            cmd = "lock" if payload[0] else "unlock"
            return f"TX {name} seq={seq} {cmd}"
        if type_id == CMD_PAN_ONLY_ABS and len(payload) >= 8:
            pan = struct.unpack_from("<f", payload, 0)[0]
            spd, acc = struct.unpack_from("<HH", payload, 4)[0], struct.unpack_from("<HH", payload, 6)[1]
            return f"TX {name} seq={seq} pan={pan:.1f} spd={spd} acc={acc}"
        if type_id == CMD_TILT_ONLY_ABS and len(payload) >= 8:
            tilt = struct.unpack_from("<f", payload, 0)[0]
            spd, acc = struct.unpack_from("<HH", payload, 4)[0], struct.unpack_from("<HH", payload, 6)[1]
            return f"TX {name} seq={seq} tilt={tilt:.1f} spd={spd} acc={acc}"
        if type_id == CMD_PAN_ONLY_MOVE and len(payload) >= 6:
            pan = struct.unpack_from("<f", payload, 0)[0]
            sx = struct.unpack_from("<H", payload, 4)[0]
            return f"TX {name} seq={seq} pan={pan:.1f} sx={sx}"
        if type_id == CMD_TILT_ONLY_MOVE and len(payload) >= 6:
            tilt = struct.unpack_from("<f", payload, 0)[0]
            sy = struct.unpack_from("<H", payload, 4)[0]
            return f"TX {name} seq={seq} tilt={tilt:.1f} sy={sy}"
        if type_id == CMD_GET_IMU or type_id == CMD_GET_INA or type_id == CMD_GET_STATE:
            return f"TX {name} seq={seq}"
        if type_id == CMD_FEEDBACK_FLOW and len(payload) >= 1:
            return f"TX {name} seq={seq} on={payload[0]}"
        if type_id == CMD_FEEDBACK_INTERVAL and len(payload) >= 2:
            ms = struct.unpack_from("<H", payload, 0)[0]
            return f"TX {name} seq={seq} interval_ms={ms}"
        if type_id == CMD_HEARTBEAT_SET and len(payload) >= 2:
            ms = struct.unpack_from("<H", payload, 0)[0]
            return f"TX {name} seq={seq} timeout_ms={ms}"
        if type_id == CMD_PING_SERVO and len(payload) >= 1:
            return f"TX {name} seq={seq} id={payload[0]}"
        if type_id == CMD_SET_SERVO_ID and len(payload) >= 2:
            return f"TX {name} seq={seq} from={payload[0]} to={payload[1]}"
        if type_id == CMD_READ_BYTE and len(payload) >= 2:
            return f"TX {name} seq={seq} id={payload[0]} addr={payload[1]}"
        if type_id == CMD_READ_WORD and len(payload) >= 2:
            return f"TX {name} seq={seq} id={payload[0]} addr={payload[1]}"
        if type_id == CMD_WRITE_BYTE and len(payload) >= 3:
            return f"TX {name} seq={seq} id={payload[0]} addr={payload[1]} val={payload[2]}"
        if type_id == CMD_WRITE_WORD and len(payload) >= 4:
            val = struct.unpack_from("<H", payload, 2)[0]
            return f"TX {name} seq={seq} id={payload[0]} addr={payload[1]} val={val}"
        if type_id == CMD_CALIBRATE and len(payload) >= 1:
            return f"TX {name} seq={seq} id={payload[0]}"
        if type_id in (CMD_ENTER_TRACKING, CMD_ENTER_CONFIG, CMD_EXIT_CONFIG):
            return f"TX {name} seq={seq}"
        # OTA commands
        if type_id == CMD_OTA_START and len(payload) >= 5:
            size = struct.unpack_from("<I", payload, 0)[0]
            ht = payload[4]
            return f"TX {name} seq={seq} size={size} hash_type={ht}"
        if type_id == CMD_OTA_CHUNK and len(payload) >= 6:
            off = struct.unpack_from("<I", payload, 0)[0]
            ln = struct.unpack_from("<H", payload, 4)[0]
            return f"TX {name} seq={seq} offset={off} len={ln}"
        if type_id in (CMD_OTA_END, CMD_OTA_ABORT):
            return f"TX {name} seq={seq}"
    except (struct.error, IndexError):
        pass
    return f"TX {name} seq={seq} len={len(payload)}"


def format_rx_for_log(seq: int, type_id: int, payload: bytes) -> str:
    """Human-readable RX line for debug log."""
    name = RSP_NAMES.get(type_id, f"RSP_{type_id}")
    try:
        if type_id == RSP_ACK_RECEIVED:
            return f"RX {name} seq={seq}"
        if type_id == RSP_ACK_EXECUTED and len(payload) >= 8:
            fb = decode_move_feedback(payload)
            if fb:
                return f"RX {name} seq={seq} pan_pos={fb['pan_pos']} pan_load={fb['pan_load']} tilt_pos={fb['tilt_pos']} tilt_load={fb['tilt_load']}"
        if type_id == RSP_NACK and len(payload) >= 1:
            code = payload[0]
            codes = {1: "checksum", 2: "unknown_type", 3: "state_rejected", 4: "exec_failed"}
            return f"RX {name} seq={seq} code={code} ({codes.get(code, '?')})"
        if type_id == 1002 and len(payload) >= 50:
            imu = decode_imu(payload)
            if imu:
                return f"RX IMU seq={seq} roll={imu['roll']:.2f} pitch={imu['pitch']:.2f} yaw={imu['yaw']:.2f}"
        if type_id == 1003 and len(payload) >= 28:
            imu2 = decode_imu2(payload)
            if imu2:
                return f"RX IMU2 seq={seq} ax={imu2['ax']:.3f} ay={imu2['ay']:.3f} az={imu2['az']:.3f} gx={imu2['gx']:.1f} gy={imu2['gy']:.1f} gz={imu2['gz']:.1f}"
        if type_id == 1010 and len(payload) >= 21:
            ina = decode_ina(payload)
            if ina:
                return f"RX INA seq={seq} bus_v={ina['bus_v']:.3f}V current_ma={ina['current_ma']:.2f} power_mw={ina['power_mw']:.1f}"
        if type_id == 1013 and len(payload) >= 1:
            s = payload[0]
            names = {0: "idle", 1: "tracking", 2: "config"}
            return f"RX STATE seq={seq} state={s} ({names.get(s, '?')})"
        # OTA responses
        if type_id == RSP_OTA_STARTED and len(payload) >= 5:
            d = decode_ota_started(payload)
            if d:
                return f"RX OTA_STARTED seq={seq} slot={d['inactive_slot']} size={d['slot_size']}"
        if type_id == RSP_OTA_CHUNK and len(payload) >= 5:
            d = decode_ota_chunk_ack(payload)
            if d:
                return f"RX OTA_CHUNK_ACK seq={seq} written={d['bytes_written']} progress={d['progress_pct']}%"
        if type_id == RSP_OTA_DONE and len(payload) >= 1:
            d = decode_ota_done(payload)
            if d:
                return f"RX OTA_DONE seq={seq} status={d['status']}"
        if type_id == RSP_OTA_NACK and len(payload) >= 1:
            d = decode_ota_nack(payload)
            if d:
                return f"RX OTA_NACK seq={seq} error={d['error_code']} ({d['error_name']})"
        if type_id == RSP_FW_INFO and len(payload) >= 65:
            d = decode_fw_info(payload)
            if d:
                sn = f" sn={d['serial']}" if d.get("serial") is not None else ""
                mid = f" model={d['model_id']}" if d.get("model_id") is not None else ""
                return f"RX FW_INFO seq={seq} active={d['active_slot']}{sn}{mid} A={d['version_a']!r} B={d['version_b']!r}"
        if type_id == RSP_I2C_SCAN and len(payload) >= 1:
            d = decode_i2c_scan(payload)
            if d:
                return f"RX I2C_SCAN seq={seq} count={d['count']} addresses={d['hex_addresses']}"
    except (struct.error, IndexError, KeyError):
        pass
    return f"RX {name} seq={seq} len={len(payload)}"


# Build command helpers
def cmd_pan_tilt_abs(seq: int, pan: float, tilt: float, spd: int = 3400, acc: int = 100) -> bytes:
    payload = struct.pack("<ffHH", pan, tilt, spd, acc)
    return build_frame(seq, CMD_PAN_TILT_ABS, payload)


def cmd_enter_tracking(seq: int) -> bytes:
    return build_frame(seq, CMD_ENTER_TRACKING, b"")


def cmd_get_imu(seq: int) -> bytes:
    return build_frame(seq, CMD_GET_IMU, b"")


# ---------- OTA helpers ----------

def cmd_ota_start(seq: int, total_size: int, hash_type: int = OTA_HASH_CRC32, expected_hash: bytes = b"") -> bytes:
    """Build OTA_START frame. hash_type: 0=none, 1=CRC32, 2=SHA256."""
    payload = struct.pack("<IB", total_size, hash_type) + expected_hash
    return build_frame(seq, CMD_OTA_START, payload)


def cmd_ota_chunk(seq: int, offset: int, data: bytes) -> bytes:
    """Build OTA_CHUNK frame. data max ~2KB (limited by MAX_PAYLOAD - 6)."""
    payload = struct.pack("<IH", offset, len(data)) + data
    return build_frame(seq, CMD_OTA_CHUNK, payload)


def cmd_ota_end(seq: int) -> bytes:
    """Build OTA_END frame."""
    return build_frame(seq, CMD_OTA_END, b"")


def cmd_ota_abort(seq: int) -> bytes:
    """Build OTA_ABORT frame."""
    return build_frame(seq, CMD_OTA_ABORT, b"")


def decode_ota_started(payload: bytes) -> Optional[dict]:
    """Decode ACK_OTA_STARTED: inactive_slot(1), slot_size(4)."""
    if len(payload) < 5:
        return None
    return {
        "inactive_slot": payload[0],
        "slot_size": struct.unpack_from("<I", payload, 1)[0],
    }


def decode_ota_chunk_ack(payload: bytes) -> Optional[dict]:
    """Decode ACK_OTA_CHUNK: bytes_written(4), progress_pct(1)."""
    if len(payload) < 5:
        return None
    return {
        "bytes_written": struct.unpack_from("<I", payload, 0)[0],
        "progress_pct": payload[4],
    }


def decode_ota_done(payload: bytes) -> Optional[dict]:
    """Decode ACK_OTA_DONE: status(1). 0=OK."""
    if len(payload) < 1:
        return None
    return {"status": payload[0]}


def decode_ota_nack(payload: bytes) -> Optional[dict]:
    """Decode NACK_OTA: error_code(1)."""
    if len(payload) < 1:
        return None
    codes = {1: "SIZE_MISMATCH", 2: "CHECKSUM_FAIL", 3: "FLASH_ERROR", 4: "TIMEOUT", 5: "ABORTED"}
    code = payload[0]
    return {"error_code": code, "error_name": codes.get(code, "UNKNOWN")}


def cmd_get_fw_info(seq: int) -> bytes:
    """Build GET_FW_INFO frame."""
    return build_frame(seq, CMD_GET_FW_INFO, b"")


def cmd_switch_fw(seq: int, slot: int) -> bytes:
    """Build SWITCH_FW frame. slot: 0=A (ota_0), 1=B (ota_1)."""
    return build_frame(seq, CMD_SWITCH_FW, bytes([slot & 1]))


# FW_INFO payload lengths: 65 (legacy), 69 (serial), 70 (serial+model_id)
FW_INFO_PAYLOAD_WITH_MODEL = 70
FW_INFO_PAYLOAD_WITH_SERIAL = 69


def decode_fw_info(payload: bytes) -> Optional[dict]:
    """Decode FW_INFO: active_slot(1) [, serial(4)] [, model_id(1)] , version_a(32), version_b(32)."""
    base_len = 1 + FW_VERSION_LEN * 2  # 65
    if len(payload) < base_len:
        return None
    active_slot = payload[0]
    if len(payload) >= FW_INFO_PAYLOAD_WITH_MODEL:  # 70: serial + model_id
        serial = struct.unpack_from("<I", payload, 1)[0]
        model_id = payload[5]
        version_a = payload[6 : 6 + FW_VERSION_LEN].rstrip(b"\x00").decode("utf-8", errors="replace")
        version_b = payload[6 + FW_VERSION_LEN : 6 + FW_VERSION_LEN * 2].rstrip(b"\x00").decode("utf-8", errors="replace")
        return {"active_slot": active_slot, "serial": serial, "model_id": model_id, "version_a": version_a, "version_b": version_b}
    if len(payload) >= FW_INFO_PAYLOAD_WITH_SERIAL:  # 69: serial only
        serial = struct.unpack_from("<I", payload, 1)[0]
        version_a = payload[5 : 5 + FW_VERSION_LEN].rstrip(b"\x00").decode("utf-8", errors="replace")
        version_b = payload[5 + FW_VERSION_LEN : 5 + FW_VERSION_LEN * 2].rstrip(b"\x00").decode("utf-8", errors="replace")
        return {"active_slot": active_slot, "serial": serial, "model_id": None, "version_a": version_a, "version_b": version_b}
    version_a = payload[1 : 1 + FW_VERSION_LEN].rstrip(b"\x00").decode("utf-8", errors="replace")
    version_b = payload[1 + FW_VERSION_LEN : 1 + FW_VERSION_LEN * 2].rstrip(b"\x00").decode("utf-8", errors="replace")
    return {"active_slot": active_slot, "serial": None, "model_id": None, "version_a": version_a, "version_b": version_b}


# I2C Scan
def cmd_i2c_scan(seq: int) -> bytes:
    """Build I2C_SCAN frame."""
    return build_frame(seq, CMD_I2C_SCAN, b"")


# Known WHO_AM_I values for device identification
WHO_AM_I_NAMES = {
    0x47: "ICM42688",
    0x42: "ICM42688-V / INA219",
    0x6F: "ICM42605",
    0xEA: "ICM20948",
    0x05: "QMI8658",
    0x00: "AK09918",
}


def decode_i2c_scan(payload: bytes) -> Optional[dict]:
    """Decode I2C_SCAN response: count(1), addresses(count), who75[4], who00[4]."""
    if len(payload) < 1:
        return None
    count = payload[0]
    addresses = list(payload[1:1 + count]) if count > 0 else []
    hex_addresses = [f"0x{addr:02X}" for addr in addresses]
    result = {"count": count, "addresses": addresses, "hex_addresses": hex_addresses}

    # Parse WHO_AM_I for 0x68, 0x69, 0x6A, 0x6B if present
    imu_addrs = [0x68, 0x69, 0x6A, 0x6B]
    if len(payload) >= 9 + count:
        who75 = list(payload[1 + count : 5 + count])
        who00 = list(payload[5 + count : 9 + count])
        device_ids = {}
        for i, addr in enumerate(imu_addrs):
            w75 = who75[i] if i < len(who75) else 0xFF
            w00 = who00[i] if i < len(who00) else 0xFF
            name75 = WHO_AM_I_NAMES.get(w75, None)
            name00 = WHO_AM_I_NAMES.get(w00, None)
            device_ids[f"0x{addr:02X}"] = {
                "who75": w75,
                "who00": w00,
                "name75": name75,
                "name00": name00,
            }
        result["device_ids"] = device_ids
    return result

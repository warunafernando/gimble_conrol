#!/usr/bin/env python3
"""
Step-by-step auto-tester for gimbal binary protocol.
Run from tester/: python run_tests.py [PORT] [--run-destructive]
"""

import argparse
import sys
import time
from typing import List, Optional, Tuple

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)

from protocol import (
    RSP_ACK_EXECUTED,
    RSP_ACK_RECEIVED,
    RSP_NACK,
    build_frame,
    parse_frame,
    decode_move_feedback,
    CMD_GET_IMU,
    CMD_GET_INA,
    CMD_ENTER_TRACKING,
    CMD_PAN_TILT_STOP,
    CMD_PAN_TILT_ABS,
    CMD_PAN_TILT_MOVE,
    CMD_PAN_ONLY_ABS,
    CMD_TILT_ONLY_ABS,
    CMD_PAN_ONLY_MOVE,
    CMD_TILT_ONLY_MOVE,
    CMD_PAN_LOCK,
    CMD_TILT_LOCK,
    CMD_USER_CTRL,
    CMD_FEEDBACK_FLOW,
    CMD_FEEDBACK_INTERVAL,
    CMD_HEARTBEAT_SET,
    CMD_ENTER_CONFIG,
    CMD_PING_SERVO,
    CMD_READ_BYTE,
    CMD_READ_WORD,
    CMD_WRITE_BYTE,
    CMD_WRITE_WORD,
    CMD_EXIT_CONFIG,
    CMD_SET_SERVO_ID,
    CMD_CALIBRATE,
    CMD_GET_STATE,
    CMD_RESET_SAFETY,
    CMD_GET_SAFETY_STATUS,
    RSP_SAFETY_ALERT,
    FEEDBACK_SAFETY_STATUS,
    SAFETY_ERR_STALL,
    SAFETY_ERR_OVERCURRENT,
    SAFETY_ERR_MODE_CORRUPTION,
    SAFETY_ERR_THERMAL,
)

import struct

DEFAULT_TIMEOUT = 5.0
BAUD = 921600


def find_frame_start(data: bytes) -> int:
    try:
        return data.index(0x02)
    except ValueError:
        return -1


def read_frames(
    ser: serial.Serial,
    timeout: float,
    max_frames: int = 10,
) -> List[Tuple[int, int, bytes]]:
    """Read complete frames until timeout; return list of (seq, type_id, payload)."""
    rx_buf = bytearray()
    deadline = time.monotonic() + timeout
    frames: List[Tuple[int, int, bytes]] = []
    while time.monotonic() < deadline and len(frames) < max_frames:
        if ser.in_waiting:
            rx_buf.extend(ser.read(ser.in_waiting))
        if len(rx_buf) < 8:
            time.sleep(0.01)
            continue
        idx = find_frame_start(rx_buf)
        if idx < 0:
            rx_buf.clear()
            time.sleep(0.01)
            continue
        if idx > 0:
            del rx_buf[:idx]
        length = rx_buf[1]
        if length < 4 or length > 251:
            rx_buf.pop(0)
            continue
        frame_size = 4 + length
        if len(rx_buf) < frame_size:
            time.sleep(0.01)
            continue
        frame_bytes = bytes(rx_buf[:frame_size])
        result = parse_frame(frame_bytes)
        del rx_buf[:frame_size]
        if result is not None:
            frames.append(result)
        time.sleep(0.005)
    return frames


def run_command(
    ser: serial.Serial,
    seq: int,
    type_id: int,
    payload: bytes,
    expect_types: List[int],
    expect_payload_len: Optional[int] = None,
    timeout: float = DEFAULT_TIMEOUT,
) -> Tuple[bool, str]:
    """
    Send one command frame; read until we get responses matching expect_types (in order).
    expect_types e.g. [RSP_ACK_RECEIVED, RSP_ACK_EXECUTED] or [RSP_ACK_RECEIVED, 1002].
    If expect_payload_len is set, the last response must have that payload length.
    """
    frame = build_frame(seq, type_id, payload if payload else None)
    ser.reset_input_buffer()
    ser.write(frame)
    frames = read_frames(ser, timeout=timeout, max_frames=5)
    if not frames:
        return False, "timeout (no response)"
    if len(frames) < len(expect_types):
        return False, f"expected {len(expect_types)} response(s), got {len(frames)}"
    for i, exp_type in enumerate(expect_types):
        seq_r, type_r, payload_r = frames[i]
        if type_r == RSP_NACK:
            code = payload_r[0] if payload_r else 0
            return False, f"NACK code={code}"
        if type_r != exp_type:
            return False, f"expected type {exp_type}, got {type_r}"
        if seq_r != seq and exp_type != RSP_ACK_RECEIVED:
            pass  # some responses may echo seq
    if expect_payload_len is not None and frames:
        _, _, last_payload = frames[-1]
        if len(last_payload) != expect_payload_len:
            return False, f"expected payload len {expect_payload_len}, got {len(last_payload)}"
    return True, ""


def run_command_with_feedback(
    ser: serial.Serial,
    seq: int,
    type_id: int,
    payload: bytes,
    timeout: float = DEFAULT_TIMEOUT,
) -> Tuple[bool, str, Optional[dict]]:
    """Send move command; return (ok, msg, feedback_dict). feedback_dict has pan_pos, pan_load, tilt_pos, tilt_load."""
    frame = build_frame(seq, type_id, payload if payload else None)
    ser.reset_input_buffer()
    ser.write(frame)
    frames = read_frames(ser, timeout=timeout, max_frames=5)
    if not frames:
        return False, "timeout (no response)", None
    if len(frames) < 2:
        return False, f"expected ACK_RX + ACK_EXEC, got {len(frames)}", None
    for seq_r, type_r, payload_r in frames:
        if type_r == RSP_NACK:
            code = payload_r[0] if payload_r else 0
            return False, f"NACK code={code}", None
    if frames[-1][1] != RSP_ACK_EXECUTED or len(frames[-1][2]) != 8:
        return False, f"expected ACK_EXECUTED 8B, got type={frames[-1][1]} len={len(frames[-1][2])}", None
    fb = decode_move_feedback(frames[-1][2])
    return True, "", fb


def next_seq(seq: int) -> int:
    return (seq + 1) & 0xFFFF


def run_stage_1(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Connection and basic frame: GET_IMU -> ACK_RECEIVED then 1002 (50B)."""
    return run_command(
        ser, seq, CMD_GET_IMU, b"",
        expect_types=[RSP_ACK_RECEIVED, 1002],
        expect_payload_len=50,
        timeout=DEFAULT_TIMEOUT,
    )


def run_stage_2(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """GET_INA -> ACK_RECEIVED then 1010 (21B)."""
    return run_command(
        ser, seq, CMD_GET_INA, b"",
        expect_types=[RSP_ACK_RECEIVED, 1010],
        expect_payload_len=21,
        timeout=DEFAULT_TIMEOUT,
    )


def run_stage_3(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """State machine: ENTER_TRACKING, PAN_TILT_STOP -> ACK_EXECUTED each."""
    ok, msg = run_command(ser, seq, CMD_ENTER_TRACKING, b"", expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
    if not ok:
        return False, msg
    seq = next_seq(seq)
    ok, msg = run_command(ser, seq, CMD_PAN_TILT_STOP, b"", expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
    return ok, msg


# Servo position range 0-4095; center ~2048. Load (torque) typically 0-1000 or signed.
POS_MIN, POS_MAX = 0, 4095
LOAD_MIN, LOAD_MAX = -1000, 1000


def run_stage_4(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Move commands in TRACKING with small angles (5-10 deg); verify position and torque in feedback."""
    run_command(ser, seq, CMD_ENTER_TRACKING, b"", expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
    seq = next_seq(seq)

    # Small angles 5° and 10° for pan/tilt; verify position and torque in ACK_EXECUTED
    move_commands = [
        (CMD_PAN_TILT_ABS, struct.pack("<ffHH", 5.0, 5.0, 3400, 100), "pan=5 tilt=5"),
        (CMD_PAN_TILT_ABS, struct.pack("<ffHH", 10.0, 10.0, 3400, 100), "pan=10 tilt=10"),
        (CMD_PAN_TILT_MOVE, struct.pack("<ffHH", 5.0, 5.0, 3400, 3400), "pan=5 tilt=5 move"),
        (CMD_PAN_ONLY_ABS, struct.pack("<fHH", 5.0, 3400, 100), "pan=5 only"),
        (CMD_TILT_ONLY_ABS, struct.pack("<fHH", 10.0, 3400, 100), "tilt=10 only"),
        (CMD_PAN_ONLY_MOVE, struct.pack("<fH", 0.0, 3400), "pan=0 move"),
        (CMD_TILT_ONLY_MOVE, struct.pack("<fH", 0.0, 3400), "tilt=0 move"),
    ]
    for cmd_type, payload, label in move_commands:
        ok, msg, fb = run_command_with_feedback(ser, seq, cmd_type, payload, timeout=DEFAULT_TIMEOUT)
        if not ok:
            return False, f"{label} ({cmd_type}): {msg}"
        if fb is None:
            return False, f"{label}: no feedback"
        # Check position and torque in valid ranges
        for key in ("pan_pos", "tilt_pos"):
            p = fb.get(key, 0)
            if not (POS_MIN <= p <= POS_MAX):
                return False, f"{label}: {key}={p} out of range [{POS_MIN},{POS_MAX}]"
        for key in ("pan_load", "tilt_load"):
            L = fb.get(key, 0)
            if not (LOAD_MIN <= L <= LOAD_MAX):
                return False, f"{label}: {key}={L} out of range [{LOAD_MIN},{LOAD_MAX}]"
        seq = next_seq(seq)
    return True, ""


def run_stage_5(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Stop and lock: PAN_TILT_STOP, PAN_LOCK 1, TILT_LOCK 1, PAN_LOCK 0, TILT_LOCK 0."""
    for cmd_type, payload in [
        (CMD_PAN_TILT_STOP, b""),
        (CMD_PAN_LOCK, b"\x01"),
        (CMD_TILT_LOCK, b"\x01"),
        (CMD_PAN_LOCK, b"\x00"),
        (CMD_TILT_LOCK, b"\x00"),
    ]:
        ok, msg = run_command(ser, seq, cmd_type, payload, expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
        if not ok:
            return False, f"cmd {cmd_type}: {msg}"
        seq = next_seq(seq)
    return True, ""


def run_stage_6(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """User control and settings: USER_CTRL, FEEDBACK_FLOW, FEEDBACK_INTERVAL, HEARTBEAT_SET."""
    commands = [
        (CMD_USER_CTRL, struct.pack("<bbH", 0, 0, 300)),
        (CMD_FEEDBACK_FLOW, b"\x00"),
        (CMD_FEEDBACK_INTERVAL, struct.pack("<H", 100)),
        (CMD_HEARTBEAT_SET, struct.pack("<H", 0)),
    ]
    for cmd_type, payload in commands:
        ok, msg = run_command(ser, seq, cmd_type, payload, expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
        if not ok:
            return False, f"cmd {cmd_type}: {msg}"
        seq = next_seq(seq)
    return True, ""


def run_stage_7(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """ENTER_CONFIG, PING_SERVO id=1 and id=2 -> 2001 with 9-byte payload."""
    ok, msg = run_command(ser, seq, CMD_ENTER_CONFIG, b"", expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
    if not ok:
        return False, msg
    seq = next_seq(seq)
    for sid in (1, 2):
        ok, msg = run_command(
            ser, seq, CMD_PING_SERVO, bytes([sid]),
            expect_types=[RSP_ACK_RECEIVED, 2001],
            expect_payload_len=9,
            timeout=DEFAULT_TIMEOUT,
        )
        if not ok:
            return False, f"PING id={sid}: {msg}"
        seq = next_seq(seq)
    return True, ""


def run_stage_8(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Register read/write in CONFIG: READ_BYTE, READ_WORD; optional WRITE (safe addr)."""
    # READ_BYTE id=1 addr=0 (or safe)
    ok, msg = run_command(
        ser, seq, CMD_READ_BYTE, struct.pack("<BB", 1, 0),
        expect_types=[RSP_ACK_RECEIVED, 2101],
        expect_payload_len=3,
        timeout=DEFAULT_TIMEOUT,
    )
    if not ok:
        return False, f"READ_BYTE: {msg}"
    seq = next_seq(seq)
    # READ_WORD id=1 addr=42
    ok, msg = run_command(
        ser, seq, CMD_READ_WORD, struct.pack("<BB", 1, 42),
        expect_types=[RSP_ACK_RECEIVED, 2121],
        expect_payload_len=4,
        timeout=DEFAULT_TIMEOUT,
    )
    if not ok:
        return False, f"READ_WORD: {msg}"
    seq = next_seq(seq)
    # WRITE_BYTE to RAM address (e.g. 3 or 4) then read back - avoid EPROM
    # Use addr that doesn't change behavior; many servos have RAM at 0-4. Skip if risky.
    # We only check response type 2111 (3 bytes: id, addr, ok)
    ok, msg = run_command(
        ser, seq, CMD_WRITE_BYTE, struct.pack("<BBB", 1, 3, 0),
        expect_types=[RSP_ACK_RECEIVED, 2111],
        expect_payload_len=3,
        timeout=DEFAULT_TIMEOUT,
    )
    if not ok:
        return False, f"WRITE_BYTE: {msg}"
    seq = next_seq(seq)
    ok, msg = run_command(
        ser, seq, CMD_WRITE_WORD, struct.pack("<BBH", 1, 4, 0),
        expect_types=[RSP_ACK_RECEIVED, 2131],
        expect_payload_len=3,
        timeout=DEFAULT_TIMEOUT,
    )
    return ok, f"WRITE_WORD: {msg}" if not ok else ""


def run_stage_9(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """EXIT_CONFIG, ENTER_TRACKING -> ACK_EXECUTED each."""
    ok, msg = run_command(ser, seq, CMD_EXIT_CONFIG, b"", expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
    if not ok:
        return False, msg
    seq = next_seq(seq)
    ok, msg = run_command(ser, seq, CMD_ENTER_TRACKING, b"", expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED], timeout=DEFAULT_TIMEOUT)
    return ok, msg


def run_stage_10(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Get gimbal state: CMD_GET_STATE -> ACK_RECEIVED then 1013 (1B state)."""
    ok, msg = run_command(
        ser, seq, CMD_GET_STATE, b"",
        expect_types=[RSP_ACK_RECEIVED, 1013],
        expect_payload_len=1,
        timeout=DEFAULT_TIMEOUT,
    )
    if not ok:
        return False, f"GET_STATE: {msg}"
    return True, ""


def run_stage_11(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Safety status query: CMD_GET_SAFETY_STATUS -> ACK_RECEIVED then 1014 (safety status)."""
    ok, msg = run_command(
        ser, seq, CMD_GET_SAFETY_STATUS, b"",
        expect_types=[RSP_ACK_RECEIVED, FEEDBACK_SAFETY_STATUS],
        expect_payload_len=5,  # triggered(1) + reason(1) + panStall(1) + tiltStall(1) + thermal(1)
        timeout=DEFAULT_TIMEOUT,
    )
    if not ok:
        return False, f"GET_SAFETY_STATUS: {msg}"
    return True, ""


def run_stage_12(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Safety reset: CMD_RESET_SAFETY -> ACK_EXECUTED."""
    ok, msg = run_command(
        ser, seq, CMD_RESET_SAFETY, b"",
        expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED],
        timeout=DEFAULT_TIMEOUT,
    )
    if not ok:
        return False, f"RESET_SAFETY: {msg}"
    return True, ""


def run_stage_13(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Speed/acceleration validation: verify move commands accept params within limits."""
    # Ensure we're in TRACKING mode first
    ok, msg = run_command(ser, seq, CMD_ENTER_TRACKING, b"",
                          expect_types=[RSP_ACK_RECEIVED, RSP_ACK_EXECUTED],
                          timeout=DEFAULT_TIMEOUT)
    if not ok:
        return False, f"ENTER_TRACKING: {msg}"
    seq = next_seq(seq)

    # Test with valid speed (within MAX_SERVO_SPEED=4000) and accel (within MAX_SERVO_ACCEL=254)
    # PAN_TILT_ABS: pan(float), tilt(float), speed(uint16), accel(uint16)
    valid_speed = 2000
    valid_accel = 100
    payload = struct.pack("<ffHH", 0.0, 0.0, valid_speed, valid_accel)
    ok, msg, fb = run_command_with_feedback(ser, seq, CMD_PAN_TILT_ABS, payload, timeout=DEFAULT_TIMEOUT)
    if not ok:
        return False, f"Valid speed/accel test: {msg}"
    seq = next_seq(seq)

    # Test with excessive speed (should be clamped to MAX_SERVO_SPEED=4000)
    # The firmware should accept the command but clamp internally
    excessive_speed = 10000
    excessive_accel = 500
    payload = struct.pack("<ffHH", 0.0, 0.0, excessive_speed, excessive_accel)
    ok, msg, fb = run_command_with_feedback(ser, seq, CMD_PAN_TILT_ABS, payload, timeout=DEFAULT_TIMEOUT)
    if not ok:
        return False, f"Excessive speed test (should be clamped): {msg}"

    return True, ""


def run_stage_14(ser: serial.Serial, seq: int) -> Tuple[bool, str]:
    """Optional destructive: SET_SERVO_ID, CALIBRATE. Skip by default."""
    # SET_SERVO_ID changes hardware ID - only on test fixture. We skip actual change and only check response type.
    # CALIBRATE sets current position as middle - changes calibration.
    return False, "Stage 14 (destructive) skipped; use --run-destructive to enable"


def main() -> int:
    ap = argparse.ArgumentParser(description="Gimbal step-by-step auto-tester")
    ap.add_argument("port", nargs="?", default="COM3", help="Serial port (default COM3)")
    ap.add_argument("--run-destructive", action="store_true", help="Run stage 14 (SET_SERVO_ID, CALIBRATE)")
    ap.add_argument("--baud", type=int, default=BAUD, help=f"Baud rate (default {BAUD})")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open {args.port}: {e}")
        return 1

    # Brief delay after open so ESP32/port is ready for first command
    time.sleep(0.5)

    stages = [
        (1, "Connection + GET_IMU", run_stage_1),
        (2, "GET_INA", run_stage_2),
        (3, "State machine (TRACKING/STOP)", run_stage_3),
        (4, "Move commands (6)", run_stage_4),
        (5, "Stop and lock", run_stage_5),
        (6, "User + settings", run_stage_6),
        (7, "CONFIG + PING", run_stage_7),
        (8, "Register R/W", run_stage_8),
        (9, "EXIT_CONFIG + TRACKING", run_stage_9),
        (10, "Get gimbal state", run_stage_10),
        (11, "Safety status query", run_stage_11),
        (12, "Safety reset", run_stage_12),
        (13, "Speed/accel validation", run_stage_13),
    ]
    if args.run_destructive:
        stages.append((14, "Destructive (SET_ID/CALIBRATE)", run_stage_14))

    results: List[Tuple[int, str, bool, str]] = []
    seq = 1
    try:
        for stage_num, name, run_fn in stages:
            print(f"Stage {stage_num}: {name} ... ", end="", flush=True)
            ok, msg = run_fn(ser, seq)
            results.append((stage_num, name, ok, msg))
            if ok:
                print("PASS")
            else:
                print("FAIL", msg)
            seq = next_seq(seq)
            # Advance seq a bit so next stage doesn't reuse same seq
            seq = next_seq(seq)
    finally:
        ser.close()

    passed = sum(1 for _, _, ok, _ in results if ok)
    total = len(results)
    print()
    print(f"Result: {passed}/{total} passed")
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())

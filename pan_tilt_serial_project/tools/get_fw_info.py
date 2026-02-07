#!/usr/bin/env python3
"""Connect to gimbal, send GET_FW_INFO (610), print decoded result (version, serial, payload len)."""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "backend"))
import serial
from protocol import (
    build_frame,
    parse_frame,
    cmd_get_fw_info,
    decode_fw_info,
    CMD_GET_FW_INFO,
    RSP_FW_INFO,
    RSP_ACK_RECEIVED,
    STX,
)

PORT = "COM9"
BAUD = 921600
TIMEOUT = 0.5
READ_TIMEOUT = 2.0


def find_stx(data: bytes) -> int:
    try:
        return data.index(STX)
    except ValueError:
        return -1


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    print(f"Opening {port} @ {BAUD}...")
    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    ser.reset_input_buffer()
    try:
        # Send GET_FW_INFO, seq=1
        frame = cmd_get_fw_info(1)
        ser.write(frame)
        print("Sent GET_FW_INFO (610)")

        deadline = time.monotonic() + READ_TIMEOUT
        buf = bytearray()
        fw_payload = None

        while time.monotonic() < deadline:
            buf.extend(ser.read(256))
            if len(buf) < 8:
                continue
            idx = find_stx(buf)
            if idx < 0:
                buf.clear()
                continue
            if idx > 0:
                del buf[:idx]
            length = buf[1]
            if length < 4 or length > 251:
                buf.pop(0)
                continue
            frame_size = 4 + length
            if len(buf) < frame_size:
                continue
            frame_bytes = bytes(buf[:frame_size])
            del buf[:frame_size]
            result = parse_frame(frame_bytes)
            if result is None:
                continue
            seq, type_id, payload = result
            if type_id == RSP_ACK_RECEIVED:
                continue
            if type_id == RSP_FW_INFO:
                fw_payload = payload
                break

        if fw_payload is None:
            print("No FW_INFO response received (timeout or no data).")
            return 1

        n = len(fw_payload)
        print(f"FW_INFO payload length: {n} bytes (70 = with serial+model_id)")
        print(f"  Raw first 20 bytes: {fw_payload[:20].hex()}")
        print(f"  payload[0] (active_slot) = {fw_payload[0]}")
        print(f"  payload[1:5] (serial LE) = {int.from_bytes(fw_payload[1:5], 'little')}")
        if n >= 6:
            print(f"  payload[5] (model_id if 70-byte) = {fw_payload[5]} (0x63=99)")
        if n >= 38:
            va = fw_payload[6:38].rstrip(b"\x00")
            print(f"  payload[6:38] (version_a) = {va!r}")
        if n >= 70:
            vb = fw_payload[38:70].rstrip(b"\x00")
            print(f"  payload[38:70] (version_b) = {vb!r}")
        fw = decode_fw_info(fw_payload)
        if not fw:
            print("Failed to decode FW_INFO")
            return 1
        print(f"  Version A:    {fw.get('version_a', '')!r}")
        print(f"  Version B:    {fw.get('version_b', '')!r}")
        print(f"  Active slot:  {fw.get('active_slot')} (0=A, 1=B)")
        print(f"  Serial:       {fw.get('serial')} (unique per unit)")
        print(f"  Model ID:     {fw.get('model_id')} (same for all gimbals of this type)")
        if fw.get("serial") is not None:
            print("\n  OK: Serial is present.")
        else:
            print("\n  Serial is missing (device sent 65-byte legacy format).")
        return 0
    finally:
        ser.close()


if __name__ == "__main__":
    sys.exit(main())

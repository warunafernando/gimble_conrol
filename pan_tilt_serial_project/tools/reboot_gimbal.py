#!/usr/bin/env python3
"""Send SWITCH_FW (current slot) to trigger ESP32 reboot."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "backend"))
import serial
from protocol import cmd_switch_fw

PORT = "COM9"
BAUD = 921600


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else PORT
    # Slot 1 = B (or use 0 for A). Sending current slot just reboots.
    slot = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    print(f"Opening {port} @ {BAUD}, sending SWITCH_FW slot={slot} to reboot...")
    ser = serial.Serial(port, BAUD, timeout=0.5)
    try:
        frame = cmd_switch_fw(1, slot)
        ser.write(frame)
        print("Reboot command sent. ESP32 should restart shortly.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()

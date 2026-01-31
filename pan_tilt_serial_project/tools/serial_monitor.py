#!/usr/bin/env python3
"""
Raw serial monitor - prints all bytes from ESP32 UART.
Use for debugging: stop the backend first, then run this to see Serial.print output.
Usage: python serial_monitor.py --port COM9 [--baud 921600]
"""

import argparse
import sys
import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM9", help="Serial port")
    ap.add_argument("--baud", type=int, default=921600, help="Baud rate")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open {args.port}: {e}")
        print("Make sure the backend is stopped and the port is free.")
        sys.exit(1)

    print(f"Connected to {args.port} @ {args.baud}. Press Ctrl+C to exit.")
    print("Raw serial output (including Serial.print from ESP32):")
    print("-" * 50)
    try:
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                sys.stdout.buffer.write(data)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
    print("\nDisconnected.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
IMU Data Monitor for ESP32 Pan-Tilt System

Reads and displays IMU data from the ESP32 via USB serial.
The ESP32 must be running the pan_tilt_base_v0.9 firmware.

Usage:
    python imu_monitor.py --port COM7
    python imu_monitor.py --port COM7 --continuous
    python imu_monitor.py --port COM7 --rate 10
"""

import argparse
import json
import sys
import time
from datetime import datetime

import serial


def send_command(ser: serial.Serial, cmd: dict) -> None:
    """Send JSON command to ESP32."""
    line = json.dumps(cmd, separators=(",", ":")) + "\n"
    ser.write(line.encode("utf-8"))
    ser.flush()


def read_response(ser: serial.Serial, timeout: float = 1.0) -> dict:
    """Read JSON response from ESP32."""
    end_time = time.time() + timeout
    buffer = ""
    
    while time.time() < end_time:
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            try:
                buffer += data.decode("utf-8", errors="replace")
            except Exception:
                pass
            
            # Try to parse complete JSON lines
            lines = buffer.split("\n")
            buffer = lines[-1]  # Keep incomplete line in buffer
            
            for line in lines[:-1]:
                line = line.strip()
                if line and line.startswith("{"):
                    try:
                        return json.loads(line)
                    except json.JSONDecodeError:
                        continue
        
        time.sleep(0.01)
    
    return {}


def format_imu_data(data: dict) -> str:
    """Format IMU data for display."""
    if "T" not in data or data["T"] != 1002:
        return None
    
    # Extract data with defaults
    roll = data.get("r", 0.0)
    pitch = data.get("p", 0.0)
    yaw = data.get("y", 0.0)
    
    ax = data.get("ax", 0.0)
    ay = data.get("ay", 0.0)
    az = data.get("az", 0.0)
    
    gx = data.get("gx", 0.0)
    gy = data.get("gy", 0.0)
    gz = data.get("gz", 0.0)
    
    mx = data.get("mx", 0)
    my = data.get("my", 0)
    mz = data.get("mz", 0)
    
    temp = data.get("temp", 0.0)
    
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    
    # Format output
    output = f"\n[{timestamp}] IMU Data:\n"
    output += f"  Euler Angles:  Roll={roll:7.2f}°  Pitch={pitch:7.2f}°  Yaw={yaw:7.2f}°\n"
    output += f"  Accelerometer: X={ax:7.3f}  Y={ay:7.3f}  Z={az:7.3f} m/s²\n"
    output += f"  Gyroscope:     X={gx:7.3f}  Y={gy:7.3f}  Z={gz:7.3f} rad/s\n"
    output += f"  Magnetometer:  X={mx:6d}  Y={my:6d}  Z={mz:6d} uT\n"
    if temp != 0.0:
        output += f"  Temperature:   {temp:.1f}°C\n"
    
    return output


def display_single(ser: serial.Serial) -> bool:
    """Get and display IMU data once."""
    send_command(ser, {"T": 126})
    time.sleep(0.1)  # Give ESP32 time to respond
    
    response = read_response(ser, timeout=0.5)
    
    if response:
        formatted = format_imu_data(response)
        if formatted:
            print(formatted)
            return True
        else:
            print("Received response but not IMU data:", response, file=sys.stderr)
            return False
    else:
        print("No response from ESP32", file=sys.stderr)
        return False


def display_continuous(ser: serial.Serial, rate: float = 5.0) -> None:
    """Continuously display IMU data."""
    interval = 1.0 / rate
    print(f"\n{'='*70}")
    print(f"  IMU Data Monitor - Continuous Mode ({rate} Hz)")
    print(f"{'='*70}")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            start_time = time.time()
            
            # Clear line and show data
            print("\033[2K", end="")  # Clear line
            print("\033[1A" * 7, end="")  # Move cursor up 7 lines
            
            send_command(ser, {"T": 126})
            time.sleep(0.05)
            
            response = read_response(ser, timeout=0.3)
            
            if response:
                formatted = format_imu_data(response)
                if formatted:
                    print(formatted, end="")
                else:
                    print(f"[{datetime.now().strftime('%H:%M:%S')}] Waiting for IMU data...")
            else:
                print(f"[{datetime.now().strftime('%H:%M:%S')}] No response from ESP32")
            
            # Sleep to maintain rate
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")


def display_table(ser: serial.Serial, rate: float = 2.0) -> None:
    """Display IMU data in table format."""
    interval = 1.0 / rate
    
    # Print header
    print("\n" + "="*120)
    print("  IMU Data Monitor - Table Format")
    print("="*120)
    print(f"{'Time':<12} {'Roll':<8} {'Pitch':<8} {'Yaw':<8} {'AccX':<8} {'AccY':<8} {'AccZ':<8} {'GyroX':<8} {'GyroY':<8} {'GyroZ':<8} {'MagX':<7} {'MagY':<7} {'MagZ':<7}")
    print("-"*120)
    
    try:
        while True:
            send_command(ser, {"T": 126})
            time.sleep(0.05)
            
            response = read_response(ser, timeout=0.3)
            
            if response and response.get("T") == 1002:
                timestamp = datetime.now().strftime("%H:%M:%S")
                print(f"{timestamp:<12} "
                      f"{response.get('r', 0):<8.2f} "
                      f"{response.get('p', 0):<8.2f} "
                      f"{response.get('y', 0):<8.2f} "
                      f"{response.get('ax', 0):<8.3f} "
                      f"{response.get('ay', 0):<8.3f} "
                      f"{response.get('az', 0):<8.3f} "
                      f"{response.get('gx', 0):<8.3f} "
                      f"{response.get('gy', 0):<8.3f} "
                      f"{response.get('gz', 0):<8.3f} "
                      f"{response.get('mx', 0):<7d} "
                      f"{response.get('my', 0):<7d} "
                      f"{response.get('mz', 0):<7d}")
            
            time.sleep(interval)
    
    except KeyboardInterrupt:
        print("\n\nStopped by user")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Monitor IMU data from ESP32 Pan-Tilt system via USB serial"
    )
    parser.add_argument(
        "--port", 
        required=True, 
        help="Serial port (e.g., COM7 on Windows, /dev/ttyUSB0 on Linux)"
    )
    parser.add_argument(
        "--baud", 
        type=int, 
        default=115200, 
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--continuous", 
        action="store_true", 
        help="Continuously display IMU data"
    )
    parser.add_argument(
        "--table", 
        action="store_true", 
        help="Display data in table format (implies --continuous)"
    )
    parser.add_argument(
        "--rate", 
        type=float, 
        default=5.0, 
        help="Update rate in Hz for continuous mode (default: 5.0)"
    )
    
    args = parser.parse_args()
    
    try:
        with serial.Serial(args.port, args.baud, timeout=0.5) as ser:
            # Give ESP32 time to initialize
            time.sleep(0.5)
            ser.reset_input_buffer()
            
            print(f"Connected to {args.port} at {args.baud} baud")
            print("Requesting IMU data from ESP32...\n")
            
            if args.table:
                display_table(ser, args.rate)
            elif args.continuous:
                display_continuous(ser, args.rate)
            else:
                # Single read
                success = display_single(ser)
                return 0 if success else 1
    
    except serial.SerialException as e:
        print(f"Serial port error: {e}", file=sys.stderr)
        print(f"\nMake sure:")
        print(f"  1. The ESP32 is connected to {args.port}")
        print(f"  2. The port is not being used by another program")
        print(f"  3. The correct port is specified")
        return 1
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 0
    
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())

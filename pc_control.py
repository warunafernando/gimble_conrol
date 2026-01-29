import argparse
import json
import sys
import time

import serial


def _write_json_line(ser: serial.Serial, obj: dict) -> None:
    line = json.dumps(obj, separators=(",", ":")) + "\n"
    ser.write(line.encode("utf-8"))
    ser.flush()


def _read_available(ser: serial.Serial, max_seconds: float) -> str:
    """Read whatever is available for up to max_seconds (best-effort)."""
    end = time.time() + max_seconds
    chunks: list[str] = []
    while time.time() < end:
        n = ser.in_waiting
        if n:
            data = ser.read(n)
            try:
                chunks.append(data.decode("utf-8", errors="replace"))
            except Exception:
                chunks.append(repr(data))
        else:
            time.sleep(0.02)
    return "".join(chunks).strip()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Control Waveshare 2-Axis Pan-Tilt (General Driver for Robots) via USB serial JSON commands."
    )
    p.add_argument("--port", required=True, help="Serial port, e.g. COM7 (Windows) or /dev/ttyUSB0 (Linux)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout seconds (default: 0.2)")
    p.add_argument("--read", action="store_true", help="Print any response text after sending")

    cmd = p.add_mutually_exclusive_group(required=True)
    cmd.add_argument("--json", help='Send raw JSON string, e.g. \'{"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}\'')
    cmd.add_argument("--center", action="store_true", help="Center gimbal (T=133, X=0, Y=0)")
    cmd.add_argument("--set", action="store_true", help="Set absolute gimbal angles (T=133)")
    cmd.add_argument("--move", action="store_true", help="Continuous gimbal move (T=134)")
    cmd.add_argument("--stop", action="store_true", help="Stop gimbal movement (T=135)")
    cmd.add_argument("--torque", choices=["on", "off"], help="Bus servo torque lock (T=210 cmd=1|0)")
    cmd.add_argument("--set-id", action="store_true", help="Set bus servo ID (T=501 raw->new)")
    cmd.add_argument("--set-middle", action="store_true", help="Set bus servo middle position (T=502 id=1|2)")

    # Common numeric params used by multiple commands
    p.add_argument("--x", type=float, help="Pan angle X in degrees (-180..180)")
    p.add_argument("--y", type=float, help="Tilt angle Y in degrees (-30..90)")
    p.add_argument("--spd", type=int, default=0, help="Speed (0=fastest for T=133). Default: 0")
    p.add_argument("--acc", type=int, default=0, help="Acceleration (0=fastest for T=133). Default: 0")
    p.add_argument("--sx", type=int, default=300, help="X speed for T=134. Default: 300")
    p.add_argument("--sy", type=int, default=300, help="Y speed for T=134. Default: 300")

    # Servo ID / calibration params
    p.add_argument("--raw", type=int, help="Old servo ID (default 1)")
    p.add_argument("--new", type=int, help="New servo ID")
    p.add_argument("--id", type=int, choices=[1, 2], help="Servo id (1=tilt, 2=pan) for middle-position set")

    return p.parse_args()


def main() -> int:
    args = parse_args()

    try:
        with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
            # give the CP2102/ESP32 a moment after opening the port
            time.sleep(0.2)
            ser.reset_input_buffer()

            if args.json:
                try:
                    obj = json.loads(args.json)
                    if not isinstance(obj, dict):
                        raise ValueError("JSON must be an object")
                except Exception as e:
                    print(f"Invalid --json payload: {e}", file=sys.stderr)
                    return 2
                _write_json_line(ser, obj)

            elif args.center:
                _write_json_line(ser, {"T": 133, "X": 0, "Y": 0, "SPD": 0, "ACC": 0})

            elif args.set:
                if args.x is None or args.y is None:
                    print("--set requires --x and --y", file=sys.stderr)
                    return 2
                x = max(-180, min(180, float(args.x)))
                y = max(-30, min(90, float(args.y)))
                _write_json_line(ser, {"T": 133, "X": x, "Y": y, "SPD": int(args.spd), "ACC": int(args.acc)})

            elif args.move:
                if args.x is None or args.y is None:
                    print("--move requires --x and --y", file=sys.stderr)
                    return 2
                x = max(-180, min(180, float(args.x)))
                y = max(-30, min(90, float(args.y)))
                _write_json_line(ser, {"T": 134, "X": x, "Y": y, "SX": int(args.sx), "SY": int(args.sy)})

            elif args.stop:
                _write_json_line(ser, {"T": 135})

            elif args.torque:
                cmd_val = 1 if args.torque == "on" else 0
                _write_json_line(ser, {"T": 210, "cmd": cmd_val})

            elif args.set_id:
                if args.raw is None or args.new is None:
                    print("--set-id requires --raw and --new (e.g. --raw 1 --new 2)", file=sys.stderr)
                    return 2
                _write_json_line(ser, {"T": 501, "raw": int(args.raw), "new": int(args.new)})

            elif args.set_middle:
                if args.id is None:
                    print("--set-middle requires --id 1|2", file=sys.stderr)
                    return 2
                _write_json_line(ser, {"T": 502, "id": int(args.id)})

            else:
                print("No command selected (unexpected).", file=sys.stderr)
                return 2

            if args.read:
                resp = _read_available(ser, max_seconds=0.6)
                if resp:
                    print(resp)

    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


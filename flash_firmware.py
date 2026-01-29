import argparse
import os
import subprocess
import sys
from typing import List, Tuple


def _parse_segment(seg: str) -> Tuple[str, str]:
    """
    Parse OFFSET:FILE (OFFSET can be decimal or hex like 0x1000).
    We keep OFFSET as a string because esptool accepts both forms.
    """
    if ":" not in seg:
        raise ValueError("Segment must be in the form OFFSET:FILE (e.g. 0x1000:bootloader.bin)")
    off, path = seg.split(":", 1)
    off = off.strip()
    path = path.strip().strip('"')
    if not off:
        raise ValueError("Segment OFFSET is empty")
    if not path:
        raise ValueError("Segment FILE is empty")
    return off, path


def _run(cmd: List[str]) -> int:
    print("Running:")
    print("  " + " ".join(cmd))
    proc = subprocess.run(cmd)
    return proc.returncode


def main() -> int:
    p = argparse.ArgumentParser(
        description="Flash firmware to Waveshare General Driver for Robots (ESP32) using esptool."
    )
    p.add_argument("--port", required=True, help="COM port, e.g. COM9")
    p.add_argument("--chip", default="esp32", help="Chip type for esptool (default: esp32)")
    p.add_argument("--baud", type=int, default=921600, help="Baud rate (default: 921600)")
    p.add_argument(
        "--erase",
        action="store_true",
        help="Erase flash before writing (recommended when changing firmware)",
    )

    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--image",
        help="Flash a single firmware image at offset 0x0 (merged/combined bin).",
    )
    mode.add_argument(
        "--segment",
        action="append",
        default=[],
        help="Flash a segment in the form OFFSET:FILE (repeatable). Example: --segment 0x1000:bootloader.bin",
    )

    args = p.parse_args()

    # Build write list
    pairs: List[Tuple[str, str]] = []
    if args.image:
        pairs = [("0x0", args.image)]
    else:
        try:
            pairs = [_parse_segment(s) for s in args.segment]
        except ValueError as e:
            print(f"Invalid --segment: {e}", file=sys.stderr)
            return 2

    # Validate files exist
    missing = [path for _, path in pairs if not os.path.exists(path)]
    if missing:
        print("These firmware file(s) were not found:", file=sys.stderr)
        for m in missing:
            print(f"  - {m}", file=sys.stderr)
        return 2

    base = [sys.executable, "-m", "esptool", "--chip", args.chip, "--port", args.port, "--baud", str(args.baud)]

    # Optional erase
    if args.erase:
        rc = _run(base + ["erase_flash"])
        if rc != 0:
            return rc

    # Write firmware
    write = base + [
        "write_flash",
        "-z",
        "--flash_mode",
        "dio",
        "--flash_freq",
        "40m",
        "--flash_size",
        "detect",
    ]
    for off, path in pairs:
        write += [off, path]

    return _run(write)


if __name__ == "__main__":
    raise SystemExit(main())


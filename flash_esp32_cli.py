#!/usr/bin/env python3
"""
ESP32 Flash Tool - Command Line Interface
A command-line alternative to the Waveshare GUI flash tool.

This tool can flash:
- Custom firmware from PlatformIO builds
- Waveshare factory firmware (if provided)
- Works with manual download mode entry (no DTR/RTS required)
"""

import argparse
import os
import platform
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple


def find_esptool() -> Optional[str]:
    """Find esptool - try PlatformIO's version first, then system Python."""
    # Try PlatformIO's esptool first
    home = Path.home()
    if platform.system() == "Windows":
        pio_esptool = home / ".platformio" / "packages" / "tool-esptoolpy" / "esptool.py"
    else:
        pio_esptool = home / ".platformio" / "penv" / "bin" / "esptool.py"
    
    if pio_esptool.exists():
        return str(pio_esptool)
    
    # Try system Python esptool module
    try:
        result = subprocess.run(
            [sys.executable, "-m", "esptool", "--help"],
            capture_output=True,
            timeout=5
        )
        if result.returncode == 0:
            return None  # Use -m esptool
    except (subprocess.TimeoutExpired, FileNotFoundError):
        pass
    
    return None


def run_esptool(
    esptool_path: Optional[str],
    port: str,
    baud: int,
    chip: str,
    segments: List[Tuple[str, str]],
    erase: bool = False,
    before: str = "default_reset",
    after: str = "hard_reset",
    retries: int = 3,
) -> int:
    """Run esptool to flash firmware."""
    
    # Build base command
    if esptool_path:
        base = [sys.executable, esptool_path]
    else:
        base = [sys.executable, "-m", "esptool"]
    
    base.extend([
        "--chip", chip,
        "--port", port,
        "--baud", str(baud),
        "--before", before,
        "--after", after,
    ])
    
    # Erase flash if requested
    if erase:
        print("Erasing flash...")
        erase_cmd = base + ["erase_flash"]
        result = subprocess.run(erase_cmd)
        if result.returncode != 0:
            print("ERROR: Flash erase failed!", file=sys.stderr)
            return result.returncode
    
    # Build write_flash command
    write_cmd = base + [
        "write_flash",
        "-z",
        "--flash_mode", "dio",
        "--flash_freq", "80m",
        "--flash_size", "detect",
    ]
    
    # Add segments
    for offset, filepath in segments:
        write_cmd.extend([offset, filepath])
    
    # Retry logic
    for attempt in range(1, retries + 1):
        if attempt > 1:
            print(f"\nRetry attempt {attempt} of {retries}...")
            if before == "no_reset":
                print("Make sure ESP32 is still in download mode!")
            time.sleep(2)
        
        print(f"\nFlashing firmware to {port} at {baud} baud...")
        print("Segments:")
        for offset, filepath in segments:
            print(f"  {offset}: {os.path.basename(filepath)}")
        
        result = subprocess.run(write_cmd)
        
        if result.returncode == 0:
            print("\n[SUCCESS] Flash completed successfully!")
            return 0
        else:
            print(f"\n[FAILED] Flash failed (attempt {attempt}/{retries})")
            if attempt < retries:
                print("Waiting before retry...")
    
    print("\n[ERROR] All flash attempts failed!", file=sys.stderr)
    return 1


def flash_platformio_firmware(
    port: str,
    project_dir: str = "pan_tilt_base_v0.9",
    baud: int = 115200,
    erase: bool = False,
    manual_mode: bool = False,
) -> int:
    """Flash firmware from PlatformIO build directory."""
    
    project_path = Path(project_dir)
    build_dir = project_path / ".pio" / "build" / "esp32dev"
    
    if not build_dir.exists():
        print(f"ERROR: Build directory not found: {build_dir}", file=sys.stderr)
        print("Please compile the firmware first with: pio run", file=sys.stderr)
        return 1
    
    # Find firmware files
    bootloader = build_dir / "bootloader.bin"
    partitions = build_dir / "partitions.bin"
    firmware = build_dir / "firmware.bin"
    
    # Find boot_app0.bin
    home = Path.home()
    if platform.system() == "Windows":
        boot_app0 = home / ".platformio" / "packages" / "framework-arduinoespressif32" / "tools" / "partitions" / "boot_app0.bin"
    else:
        boot_app0 = home / ".platformio" / "packages" / "framework-arduinoespressif32" / "tools" / "partitions" / "boot_app0.bin"
    
    # Check files exist
    missing = []
    for name, path in [
        ("bootloader", bootloader),
        ("partitions", partitions),
        ("boot_app0", boot_app0),
        ("firmware", firmware),
    ]:
        if not path.exists():
            missing.append(f"{name}: {path}")
    
    if missing:
        print("ERROR: Missing firmware files:", file=sys.stderr)
        for m in missing:
            print(f"  - {m}", file=sys.stderr)
        return 1
    
    # Prepare segments
    segments = [
        ("0x1000", str(bootloader)),
        ("0x8000", str(partitions)),
        ("0xe000", str(boot_app0)),
        ("0x10000", str(firmware)),
    ]
    
    # Find esptool
    esptool_path = find_esptool()
    if esptool_path:
        print(f"Using esptool: {esptool_path}")
    else:
        print("Using system esptool module")
    
    # Configure reset mode
    before_mode = "no_reset" if manual_mode else "default_reset"
    
    if manual_mode:
        print("\n*** MANUAL DOWNLOAD MODE ***")
        print("Please put ESP32 into download mode:")
        print("  1. Hold BOOT button")
        print("  2. Press and release RESET button")
        print("  3. Release BOOT button")
        print("\nStarting flash in 3 seconds...")
        time.sleep(3)
    
    return run_esptool(
        esptool_path=esptool_path,
        port=port,
        baud=baud,
        chip="esp32",
        segments=segments,
        erase=erase,
        before=before_mode,
        after="hard_reset",
        retries=3,
    )


def flash_custom_firmware(
    port: str,
    segments: List[str],
    baud: int = 115200,
    erase: bool = False,
    manual_mode: bool = False,
) -> int:
    """Flash custom firmware with specified segments."""
    
    # Parse segments
    parsed_segments = []
    for seg in segments:
        if ":" not in seg:
            print(f"ERROR: Invalid segment format: {seg}", file=sys.stderr)
            print("Expected format: OFFSET:FILE (e.g. 0x1000:bootloader.bin)", file=sys.stderr)
            return 1
        
        offset, filepath = seg.split(":", 1)
        offset = offset.strip()
        filepath = filepath.strip().strip('"').strip("'")
        
        if not os.path.exists(filepath):
            print(f"ERROR: File not found: {filepath}", file=sys.stderr)
            return 1
        
        parsed_segments.append((offset, filepath))
    
    # Find esptool
    esptool_path = find_esptool()
    
    # Configure reset mode
    before_mode = "no_reset" if manual_mode else "default_reset"
    
    if manual_mode:
        print("\n*** MANUAL DOWNLOAD MODE ***")
        print("Please put ESP32 into download mode:")
        print("  1. Hold BOOT button")
        print("  2. Press and release RESET button")
        print("  3. Release BOOT button")
        print("\nStarting flash in 3 seconds...")
        time.sleep(3)
    
    return run_esptool(
        esptool_path=esptool_path,
        port=port,
        baud=baud,
        chip="esp32",
        segments=parsed_segments,
        erase=erase,
        before=before_mode,
        after="hard_reset",
        retries=3,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="ESP32 Flash Tool - Command-line alternative to Waveshare GUI tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Flash PlatformIO firmware (auto-detect files)
  python flash_esp32_cli.py --port COM9 --platformio

  # Flash with manual download mode (if DTR/RTS doesn't work)
  python flash_esp32_cli.py --port COM9 --platformio --manual

  # Flash custom firmware segments
  python flash_esp32_cli.py --port COM9 --segment 0x1000:bootloader.bin --segment 0x10000:app.bin

  # Flash with erase and custom baud rate
  python flash_esp32_cli.py --port COM9 --platformio --erase --baud 921600
        """
    )
    
    parser.add_argument("--port", required=True, help="COM port (e.g. COM9 on Windows, /dev/ttyUSB0 on Linux)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200, max: 921600)")
    parser.add_argument("--erase", action="store_true", help="Erase flash before writing")
    parser.add_argument("--manual", action="store_true", help="Manual download mode (no DTR/RTS, you put ESP32 in download mode)")
    
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument(
        "--platformio",
        action="store_true",
        help="Flash firmware from PlatformIO build directory"
    )
    mode_group.add_argument(
        "--segment",
        action="append",
        dest="segments",
        help="Flash segment in format OFFSET:FILE (can be repeated multiple times)"
    )
    
    parser.add_argument(
        "--project-dir",
        default="pan_tilt_base_v0.9",
        help="PlatformIO project directory (default: pan_tilt_base_v0.9)"
    )
    
    args = parser.parse_args()
    
    # Validate baud rate
    if args.baud > 921600:
        print("WARNING: Baud rate > 921600 may not work reliably", file=sys.stderr)
    
    # Flash based on mode
    if args.platformio:
        return flash_platformio_firmware(
            port=args.port,
            project_dir=args.project_dir,
            baud=args.baud,
            erase=args.erase,
            manual_mode=args.manual,
        )
    else:
        if not args.segments:
            print("ERROR: --segment required when not using --platformio", file=sys.stderr)
            return 1
        
        return flash_custom_firmware(
            port=args.port,
            segments=args.segments,
            baud=args.baud,
            erase=args.erase,
            manual_mode=args.manual,
        )


if __name__ == "__main__":
    sys.exit(main())

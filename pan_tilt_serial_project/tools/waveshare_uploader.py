#!/usr/bin/env python3
"""
Waveshare-Compatible ESP32 Uploader
A Python tool that mimics the Waveshare Flash Download Tool behavior
for reliable ESP32 firmware uploads on the General Driver for Robots board.
"""

import sys
import os
import time
import serial
import argparse
from pathlib import Path

# Try to import esptool
try:
    import esptool
    from esptool import ESP32ROM
except ImportError:
    print("ERROR: esptool not found!")
    print("Install it with: pip install esptool")
    sys.exit(1)


class WaveshareUploader:
    """Uploader that works with Waveshare auto-download circuit"""
    
    def __init__(self, port, baud=115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.esp = None
    
    def activate_auto_download(self, max_retries=3):
        """Activate the auto-download circuit using DTR/RTS sequence"""
        print("=" * 60)
        print("Activating Waveshare auto-download circuit...")
        print("=" * 60)
        
        for retry in range(max_retries):
            try:
                # Open serial port for DTR/RTS control
                if not self.ser or not self.ser.is_open:
                    self.ser = serial.Serial(self.port, self.baud, timeout=1)
                break  # Success, exit retry loop
            except serial.SerialException as e:
                error_msg = str(e)
                if ("PermissionError" in error_msg or "Access is denied" in error_msg) and retry < max_retries - 1:
                    print(f"Port locked, waiting 2 seconds and retrying ({retry + 1}/{max_retries})...")
                    time.sleep(2)
                    continue
                else:
                    if "PermissionError" in error_msg or "Access is denied" in error_msg:
                        print(f"ERROR: Serial port {self.port} is already in use!")
                        print("  Please close any programs using this port:")
                        print("    - Serial monitors (Arduino IDE, PlatformIO, etc.)")
                        print("    - Other terminal programs")
                        print("    - Previous upload attempts")
                        print("  Or try unplugging and replugging the USB cable")
                    else:
                        print(f"ERROR: Could not open serial port {self.port}: {e}")
                    return False
            except Exception as e:
                print(f"ERROR: Failed to activate auto-download circuit: {e}")
                return False
        
        try:
            
            # Sequence that mimics Waveshare tool behavior
            print("Sending DTR/RTS sequence...")
            
            # Step 1: Reset (EN low) with GPIO0 high
            self.ser.setDTR(False)  # DTR low = EN low (reset)
            self.ser.setRTS(True)   # RTS high = GPIO0 high
            time.sleep(0.1)
            
            # Step 2: Release reset with GPIO0 high (normal boot)
            self.ser.setDTR(True)   # DTR high = EN high (release reset)
            self.ser.setRTS(True)   # Keep RTS high
            time.sleep(0.05)
            
            # Step 3: Set GPIO0 low (download mode) while resetting
            self.ser.setDTR(False)  # DTR low = EN low (reset again)
            self.ser.setRTS(False)  # RTS low = GPIO0 low (download mode)
            time.sleep(0.15)
            
            # Step 4: Release reset with GPIO0 low (now in download mode)
            self.ser.setDTR(True)   # DTR high = EN high (release reset)
            self.ser.setRTS(False)   # Keep RTS low (GPIO0 low = download mode)
            
            # Wait for auto-download circuit to fully stabilize
            print("Waiting for ESP32 to enter download mode...")
            time.sleep(1.0)  # Longer wait for circuit to stabilize
            
            print("[OK] Auto-download circuit activated")
            print()
            return True
            
        except serial.SerialException as e:
            error_msg = str(e)
            if "PermissionError" in error_msg or "Access is denied" in error_msg:
                print(f"ERROR: Serial port {self.port} is already in use!")
                print("  Please close any programs using this port:")
                print("    - Serial monitors (Arduino IDE, PlatformIO, etc.)")
                print("    - Other terminal programs")
                print("    - Previous upload attempts")
            else:
                print(f"ERROR: Could not open serial port {self.port}: {e}")
            return False
        except Exception as e:
            print(f"ERROR: Failed to activate auto-download circuit: {e}")
            return False
    
    def connect(self, retries=5):
        """Connect to ESP32 in download mode"""
        print("Connecting to ESP32...")
        
        for attempt in range(retries):
            try:
                print(f"  Attempt {attempt + 1}/{retries}...")
                # Use existing serial port if available, otherwise open new one
                if self.ser and self.ser.is_open:
                    ser = self.ser
                    # Reset DTR/RTS to ensure clean state
                    ser.setDTR(True)
                    ser.setRTS(False)
                else:
                    ser = serial.Serial(self.port, self.baud, timeout=5)
                self.esp = ESP32ROM(ser)
                
                # Connect - use no_reset if manual mode, otherwise default_reset
                try:
                    if hasattr(self, '_manual_mode') and self._manual_mode:
                        print("    Connecting to ESP32 (manual mode, no_reset)...")
                        self.esp.connect('no_reset', False)
                        print("    Connection established")
                    else:
                        print("    Connecting to ESP32 (using default_reset)...")
                        self.esp.connect('default_reset', False)
                        print("    Connection established")
                except Exception as connect_err:
                    ser.close()
                    self.esp = None
                    if attempt < retries - 1:
                        print(f"    Connect failed: {str(connect_err)[:60]}")
                        if hasattr(self, '_manual_mode') and self._manual_mode:
                            print("    Make sure ESP32 is in download mode and retrying...")
                            time.sleep(1)
                            # Don't activate auto-download in manual mode
                        else:
                            print("    Retrying...")
                            time.sleep(0.5)
                            self.activate_auto_download()
                        continue
                    else:
                        print(f"[ERROR] Cannot connect to ESP32. It may not be in download mode.")
                        print(f"  Error: {connect_err}")
                        raise
                
                # Verify connection by reading chip description
                print("    Verifying connection...")
                try:
                    chip_desc = self.esp.get_chip_description()
                    print(f"[OK] Connected! Chip: {chip_desc}")
                    return True
                except Exception as e:
                    if not (self.ser and self.ser.is_open):
                        ser.close()
                    self.esp = None
                    if attempt < retries - 1:
                        print(f"    Chip info read failed: {str(e)[:60]}")
                        print("    Retrying...")
                        time.sleep(0.5)
                        # Try activating auto-download again
                        self.activate_auto_download()
                        continue
                    else:
                        raise
                        
            except Exception as e:
                if self.esp:
                    try:
                        if not (self.ser and self.ser.is_open):
                            self.esp._port.close()
                    except:
                        pass
                    self.esp = None
                
                if attempt < retries - 1:
                    print(f"    Connection error: {str(e)[:60]}")
                    print("    Retrying...")
                    time.sleep(0.5)
                    # Try activating auto-download again (only if not manual mode)
                    if not (hasattr(self, '_manual_mode') and self._manual_mode):
                        self.activate_auto_download()
                else:
                    print(f"[ERROR] Failed to connect after {retries} attempts")
                    print(f"  Last error: {e}")
                    return False
        
        return False
    
    def upload_firmware(self, firmware_path, address=0x10000):
        """Upload firmware to ESP32 using esptool command line (like flash_esp32_cli.py)"""
        firmware_path = Path(firmware_path)
        if not firmware_path.exists():
            print(f"ERROR: Firmware file not found: {firmware_path}")
            return False
        
        print("=" * 60)
        print("Uploading firmware...")
        print("=" * 60)
        print(f"File: {firmware_path.name}")
        print(f"Size: {firmware_path.stat().st_size:,} bytes")
        print(f"Address: 0x{address:X}")
        print()
        
        # Close our serial port - esptool will open its own
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
        
        # Use esptool command line (same as successful flash_esp32_cli.py)
        import subprocess
        import sys
        
        # Find esptool
        esptool_cmd = [sys.executable, "-m", "esptool"]
        
        before_mode = "no_reset" if (hasattr(self, '_manual_mode') and self._manual_mode) else "default_reset"
        
        cmd = esptool_cmd + [
            "--chip", "esp32",
            "--port", self.port,
            "--baud", str(self.baud),
            "--before", before_mode,
            "--after", "hard_reset",
            "write_flash",
            "-z",
            "--flash_mode", "dio",
            "--flash_freq", "80m",
            "--flash_size", "detect",
            f"0x{address:X}",
            str(firmware_path)
        ]
        
        try:
            print("    Running esptool...")
            result = subprocess.run(cmd, check=True)
            print("[OK] Firmware uploaded successfully!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] Upload failed: {e}")
            return False
        except Exception as e:
            print(f"[ERROR] Upload failed: {e}")
            return False
    
    def upload_bootloader(self, bootloader_path, address=0x1000):
        """Upload bootloader using esptool command line"""
        bootloader_path = Path(bootloader_path)
        if not bootloader_path.exists():
            print(f"WARNING: Bootloader file not found: {bootloader_path}")
            return False
        
        print(f"Uploading bootloader to 0x{address:X}...")
        return self._upload_file_esptool(bootloader_path, address, "bootloader")
    
    def upload_partitions(self, partitions_path, address=0x8000):
        """Upload partition table using esptool command line"""
        partitions_path = Path(partitions_path)
        if not partitions_path.exists():
            print(f"WARNING: Partitions file not found: {partitions_path}")
            return False
        
        print(f"Uploading partitions to 0x{address:X}...")
        return self._upload_file_esptool(partitions_path, address, "partitions")
    
    def _upload_file_esptool(self, file_path, address, name):
        """Upload a file using esptool command line"""
        import subprocess
        import sys
        
        before_mode = "no_reset" if (hasattr(self, '_manual_mode') and self._manual_mode) else "default_reset"
        
        cmd = [
            sys.executable, "-m", "esptool",
            "--chip", "esp32",
            "--port", self.port,
            "--baud", str(self.baud),
            "--before", before_mode,
            "--after", "hard_reset",
            "write_flash",
            "-z",
            "--flash_mode", "dio",
            "--flash_freq", "80m",
            "--flash_size", "detect",
            f"0x{address:X}",
            str(file_path)
        ]
        
        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            print(f"[OK] {name.capitalize()} uploaded")
            return True
        except subprocess.CalledProcessError as e:
            print(f"[ERROR] {name.capitalize()} upload failed")
            if e.stderr:
                print(f"  Error: {e.stderr[:200]}")
            return False
    
    def reset(self):
        """Reset ESP32"""
        if self.esp:
            print("Resetting ESP32...")
            try:
                self.esp.hard_reset()
                print("[OK] ESP32 reset")
            except:
                pass
        
        if self.ser and self.ser.is_open:
            self.ser.close()
    
    def close(self):
        """Close connections"""
        if self.esp:
            try:
                # Only close if it's not the same as self.ser
                if not (self.ser and self.esp._port == self.ser):
                    self.esp._port.close()
            except:
                pass
            self.esp = None
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
            self.ser = None


def main():
    parser = argparse.ArgumentParser(
        description='Waveshare-Compatible ESP32 Uploader',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Upload firmware only
  python waveshare_uploader.py -p COM9 firmware.bin
  
  # Upload all components
  python waveshare_uploader.py -p COM9 \\
      --bootloader bootloader.bin \\
      --partitions partitions.bin \\
      --firmware firmware.bin
  
  # Use build directory from PlatformIO
  python waveshare_uploader.py -p COM9 \\
      --build-dir .pio/build/esp32dev
        """
    )
    
    parser.add_argument('-p', '--port', required=True,
                       help='Serial port (e.g., COM9 or /dev/ttyUSB0)')
    parser.add_argument('-b', '--baud', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    
    # Firmware file
    parser.add_argument('firmware', nargs='?',
                       help='Firmware binary file (or use --firmware)')
    parser.add_argument('--firmware', dest='firmware_opt',
                       help='Firmware binary file')
    parser.add_argument('--firmware-addr', type=lambda x: int(x, 0), default=0x10000,
                       help='Firmware address (default: 0x10000)')
    
    # Bootloader and partitions
    parser.add_argument('--bootloader',
                       help='Bootloader binary file')
    parser.add_argument('--bootloader-addr', type=lambda x: int(x, 0), default=0x1000,
                       help='Bootloader address (default: 0x1000)')
    parser.add_argument('--partitions',
                       help='Partitions binary file')
    parser.add_argument('--partitions-addr', type=lambda x: int(x, 0), default=0x8000,
                       help='Partitions address (default: 0x8000)')
    
    # Build directory option (PlatformIO)
    parser.add_argument('--build-dir',
                       help='PlatformIO build directory (auto-detects files)')
    
    # Manual mode option (like flash_esp32_cli.py)
    parser.add_argument('--manual', action='store_true',
                       help='Manual download mode - wait for user to put ESP32 in download mode (no DTR/RTS)')
    
    args = parser.parse_args()
    
    # Determine firmware file
    firmware_file = args.firmware_opt or args.firmware
    
    # If build-dir specified, auto-detect files
    if args.build_dir:
        build_dir = Path(args.build_dir)
        if not build_dir.exists():
            print(f"ERROR: Build directory not found: {build_dir}")
            sys.exit(1)
        
        bootloader_file = build_dir / "bootloader.bin"
        partitions_file = build_dir / "partitions.bin"
        firmware_file = build_dir / "firmware.bin"
        
        if not firmware_file.exists():
            print(f"ERROR: Firmware not found in build directory: {firmware_file}")
            sys.exit(1)
        
        print(f"Using build directory: {build_dir}")
        if bootloader_file.exists():
            args.bootloader = str(bootloader_file)
        if partitions_file.exists():
            args.partitions = str(partitions_file)
        firmware_file = str(firmware_file)
    
    if not firmware_file:
        print("ERROR: No firmware file specified")
        parser.print_help()
        sys.exit(1)
    
    # Create uploader
    uploader = WaveshareUploader(args.port, args.baud)
    uploader._manual_mode = args.manual  # Pass manual mode flag to uploader
    
    try:
        # Manual mode: wait for user to put ESP32 in download mode
        if args.manual:
            print("\n" + "=" * 60)
            print("*** MANUAL DOWNLOAD MODE ***")
            print("=" * 60)
            print("Please put ESP32 into download mode:")
            print("  1. Hold BOOT button")
            print("  2. Press and release RESET button")
            print("  3. Release BOOT button")
            print("\nStarting upload in 5 seconds...")
            print("(Put ESP32 in download mode NOW if you haven't already!)")
            for i in range(5, 0, -1):
                print(f"  {i}...")
                time.sleep(1)
            print()
        else:
            # Activate auto-download (DTR/RTS); keep port open and flash in-process so chip stays in download mode
            if not uploader.activate_auto_download():
                print("\nTroubleshooting:")
                print("1. Make sure ESP32 is connected to the correct COM port")
                print("2. Try using --manual flag for manual download mode:")
                print("   python waveshare_uploader.py -p COM6 --build-dir .pio/build/esp32dev --manual")
                sys.exit(1)
        
        # Build segments list
        segments = []
        if args.bootloader:
            segments.append((args.bootloader_addr, args.bootloader))
        if args.partitions:
            segments.append((args.partitions_addr, args.partitions))
        segments.append((args.firmware_addr, firmware_file))
        
        print("=" * 60)
        print("Uploading all firmware components...")
        print("=" * 60)
        for addr, filepath in segments:
            print(f"  0x{addr:X}: {Path(filepath).name}")
        print()
        
        # Flash using open port (no close = chip stays in download mode)
        try:
            if uploader.ser and uploader.ser.is_open:
                # Use loader with existing serial port (no close => chip stays in download mode)
                esp = ESP32ROM(uploader.ser)  # loader accepts Serial object when port is not str
                esp.connect('no_reset')
                esp.flash_set_parameters(4 * 1024 * 1024)  # 4MB default
                for addr, path in segments:
                    with open(path, 'rb') as f:
                        data = f.read()
                    num_blocks = esp.flash_begin(len(data), addr)
                    block_size = esp.FLASH_WRITE_SIZE
                    for seq in range(num_blocks):
                        start = seq * block_size
                        block = data[start:start + block_size]
                        if len(block) < block_size:
                            block += b'\xff' * (block_size - len(block))
                        esp.flash_block(block, seq)
                    esp.flash_finish(False)
                esp.hard_reset()
                uploader.ser = None
            else:
                raise RuntimeError("Serial port not open")
            print()
            print("=" * 60)
            print("[SUCCESS] All files uploaded!")
            print("=" * 60)
        except Exception as e:
            # Fallback: close port and run esptool subprocess
            uploader.close()
            time.sleep(0.2)
            import subprocess
            before_mode = "no_reset"
            cmd = [sys.executable, "-m", "esptool", "--chip", "esp32", "--port", args.port,
                   "--baud", str(args.baud), "--before", before_mode, "--after", "hard_reset",
                   "write_flash", "-z", "--flash_mode", "dio", "--flash_freq", "80m", "--flash_size", "detect"]
            for addr, filepath in segments:
                cmd.extend([f"0x{addr:X}", filepath])
            print("Running esptool subprocess (fallback)...")
            try:
                subprocess.run(cmd, check=True)
                print("=" * 60)
                print("[SUCCESS] All files uploaded!")
                print("=" * 60)
            except subprocess.CalledProcessError:
                print("=" * 60)
                print("[FAILED] Upload failed")
                print("=" * 60)
                sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\nUpload cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n\nERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        uploader.close()


if __name__ == '__main__':
    main()

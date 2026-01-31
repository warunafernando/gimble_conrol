#!/usr/bin/env python3
"""
UART OTA Firmware Upload Tool for ESP32 Pan-Tilt Controller

Uploads firmware over the existing serial protocol without requiring
manual bootloader mode (BOOT+RESET). Uses A/B partition scheme with
commit-after-checksum verification.

Usage:
    python uart_ota.py -p COM9 -f firmware.bin
    python uart_ota.py -p COM9  # Auto-find firmware from PlatformIO build

Requirements:
    - ESP32 must be running firmware with OTA support (Phase 3+)
    - pyserial installed
"""

import argparse
import os
import sys
import time
import struct
import zlib
import serial
from typing import Optional, Tuple

# Add parent directory to path for protocol import
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "backend"))

try:
    from protocol import (
        build_frame, parse_frame,
        CMD_OTA_START, CMD_OTA_CHUNK, CMD_OTA_END, CMD_OTA_ABORT,
        RSP_OTA_STARTED, RSP_OTA_CHUNK, RSP_OTA_DONE, RSP_OTA_NACK,
        RSP_ACK_RECEIVED, RSP_NACK,
        OTA_HASH_CRC32, OTA_HASH_SHA256, OTA_HASH_NONE,
        cmd_ota_start, cmd_ota_chunk, cmd_ota_end, cmd_ota_abort,
        decode_ota_started, decode_ota_chunk_ack, decode_ota_done, decode_ota_nack,
    )
except ImportError:
    # Fallback: define protocol locally if import fails
    print("Warning: Could not import protocol module, using embedded protocol")
    
    STX = 0x02
    ETX = 0x03
    MAX_PAYLOAD = 251
    
    CMD_OTA_START = 600
    CMD_OTA_CHUNK = 601
    CMD_OTA_END = 602
    CMD_OTA_ABORT = 603
    
    RSP_OTA_STARTED = 2600
    RSP_OTA_CHUNK = 2601
    RSP_OTA_DONE = 2602
    RSP_OTA_NACK = 2603
    RSP_ACK_RECEIVED = 1
    RSP_NACK = 3
    
    OTA_HASH_NONE = 0
    OTA_HASH_CRC32 = 1
    OTA_HASH_SHA256 = 2
    
    def crc8(data: bytes) -> int:
        crc = 0x00
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (0x07 ^ (crc << 1)) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
        return crc
    
    def build_frame(seq: int, type_id: int, payload: bytes = b"") -> bytes:
        length = 4 + len(payload)
        body = struct.pack("<BHH", length, seq & 0xFFFF, type_id & 0xFFFF) + payload
        checksum = crc8(body)
        return bytes([STX]) + body + bytes([checksum, ETX])
    
    def parse_frame(data: bytes) -> Optional[Tuple[int, int, bytes]]:
        if len(data) < 8 or data[0] != STX:
            return None
        length = data[1]
        if length < 4:
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
    
    def cmd_ota_start(seq: int, total_size: int, hash_type: int, expected_hash: bytes) -> bytes:
        payload = struct.pack("<IB", total_size, hash_type) + expected_hash
        return build_frame(seq, CMD_OTA_START, payload)
    
    def cmd_ota_chunk(seq: int, offset: int, data: bytes) -> bytes:
        payload = struct.pack("<IH", offset, len(data)) + data
        return build_frame(seq, CMD_OTA_CHUNK, payload)
    
    def cmd_ota_end(seq: int) -> bytes:
        return build_frame(seq, CMD_OTA_END, b"")
    
    def cmd_ota_abort(seq: int) -> bytes:
        return build_frame(seq, CMD_OTA_ABORT, b"")
    
    def decode_ota_started(payload: bytes):
        if len(payload) < 5:
            return None
        return {"inactive_slot": payload[0], "slot_size": struct.unpack_from("<I", payload, 1)[0]}
    
    def decode_ota_chunk_ack(payload: bytes):
        if len(payload) < 5:
            return None
        return {"bytes_written": struct.unpack_from("<I", payload, 0)[0], "progress_pct": payload[4]}
    
    def decode_ota_done(payload: bytes):
        if len(payload) < 1:
            return None
        return {"status": payload[0]}
    
    def decode_ota_nack(payload: bytes):
        if len(payload) < 1:
            return None
        codes = {1: "SIZE_MISMATCH", 2: "CHECKSUM_FAIL", 3: "FLASH_ERROR", 4: "TIMEOUT", 5: "ABORTED"}
        code = payload[0]
        return {"error_code": code, "error_name": codes.get(code, "UNKNOWN")}


# Constants
DEFAULT_BAUD = 921600
# MAX_PAYLOAD in protocol is 251 bytes, minus 6 bytes for offset(4)+length(2) header
CHUNK_SIZE = 240  # Safe chunk size that fits in protocol frame
RESPONSE_TIMEOUT = 5.0  # Seconds to wait for response
CHUNK_DELAY = 0.01  # Small delay between chunks to prevent buffer overflow
MAX_RETRIES = 3


class OTAUploader:
    def __init__(self, port: str, baud: int = DEFAULT_BAUD, verbose: bool = False):
        self.port = port
        self.baud = baud
        self.verbose = verbose
        self.ser: Optional[serial.Serial] = None
        self.seq = 0
        self.rx_buffer = bytearray()
    
    def _next_seq(self) -> int:
        self.seq = (self.seq + 1) & 0xFFFF
        return self.seq
    
    def _log(self, msg: str):
        if self.verbose:
            print(f"  [DEBUG] {msg}")
    
    def connect(self) -> bool:
        """Open serial port."""
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.1,
                write_timeout=2.0
            )
            # Wait for ESP32 startup messages to finish
            print(f"Connected to {self.port} at {self.baud} baud")
            print("  Waiting for ESP32 startup to complete...")
            time.sleep(1.0)
            
            # Read and discard any startup messages
            while self.ser.in_waiting > 0:
                self.ser.read(self.ser.in_waiting)
                time.sleep(0.1)
            
            # Clear buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.rx_buffer.clear()
            time.sleep(0.1)
            return True
        except serial.SerialException as e:
            print(f"Error: Could not open {self.port}: {e}")
            return False
    
    def disconnect(self):
        """Close serial port."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected")
    
    def _send_frame(self, frame: bytes) -> bool:
        """Send a frame over serial."""
        try:
            self.ser.write(frame)
            self.ser.flush()
            self._log(f"TX {len(frame)} bytes")
            return True
        except serial.SerialException as e:
            print(f"Error sending: {e}")
            return False
    
    def _recv_frame(self, timeout: float = RESPONSE_TIMEOUT) -> Optional[Tuple[int, int, bytes]]:
        """Receive and parse a frame."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Read available bytes
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.rx_buffer.extend(data)
                self._log(f"Buffer now {len(self.rx_buffer)} bytes: {bytes(self.rx_buffer[:32]).hex() if self.rx_buffer else ''}")
            
            # Try to parse frame from buffer
            if len(self.rx_buffer) >= 8:
                # Find STX (0x02)
                stx_idx = -1
                for i, b in enumerate(self.rx_buffer):
                    if b == 0x02:
                        stx_idx = i
                        break
                
                if stx_idx > 0:
                    self.rx_buffer = self.rx_buffer[stx_idx:]
                elif stx_idx < 0:
                    self.rx_buffer.clear()
                    continue
                
                # Check if we have enough bytes for the frame
                if len(self.rx_buffer) < 2:
                    time.sleep(0.01)
                    continue
                    
                length = self.rx_buffer[1]
                frame_size = 4 + length
                
                if len(self.rx_buffer) < frame_size:
                    time.sleep(0.01)
                    continue
                
                # Try to parse
                result = parse_frame(bytes(self.rx_buffer[:frame_size]))
                if result:
                    self.rx_buffer = self.rx_buffer[frame_size:]
                    self._log(f"RX type={result[1]} seq={result[0]} len={len(result[2])}")
                    return result
                else:
                    # Invalid frame, skip first byte and try again
                    self._log(f"Invalid frame, skipping byte")
                    self.rx_buffer = self.rx_buffer[1:]
            
            time.sleep(0.01)
        
        return None
    
    def _wait_for_response(self, expected_types: list, timeout: float = RESPONSE_TIMEOUT) -> Optional[Tuple[int, int, bytes]]:
        """Wait for specific response type(s)."""
        while True:
            result = self._recv_frame(timeout)
            if result is None:
                return None
            seq, type_id, payload = result
            if type_id in expected_types:
                return result
            elif type_id == RSP_NACK:
                print(f"  Received NACK")
                return result
            elif type_id == RSP_OTA_NACK:
                d = decode_ota_nack(payload)
                if d:
                    print(f"  OTA NACK: {d['error_name']}")
                return result
            # Otherwise continue waiting
    
    def upload(self, firmware_path: str, hash_type: int = OTA_HASH_CRC32) -> bool:
        """Upload firmware via UART OTA."""
        
        # Read firmware file
        if not os.path.exists(firmware_path):
            print(f"Error: Firmware file not found: {firmware_path}")
            return False
        
        with open(firmware_path, "rb") as f:
            firmware_data = f.read()
        
        total_size = len(firmware_data)
        print(f"Firmware: {firmware_path}")
        print(f"Size: {total_size} bytes ({total_size / 1024:.1f} KB)")
        
        # Compute hash
        if hash_type == OTA_HASH_CRC32:
            firmware_crc = zlib.crc32(firmware_data) & 0xFFFFFFFF
            expected_hash = struct.pack("<I", firmware_crc)
            print(f"CRC32: 0x{firmware_crc:08X}")
        elif hash_type == OTA_HASH_SHA256:
            import hashlib
            firmware_hash = hashlib.sha256(firmware_data).digest()
            expected_hash = firmware_hash
            print(f"SHA256: {firmware_hash.hex()[:16]}...")
        else:
            expected_hash = b""
            print("Hash: None (no verification)")
        
        # Step 1: Send OTA_START
        print("\n[1/3] Sending OTA_START...")
        frame = cmd_ota_start(self._next_seq(), total_size, hash_type, expected_hash)
        if not self._send_frame(frame):
            return False
        
        # Wait for ACK_OTA_STARTED
        result = self._wait_for_response([RSP_OTA_STARTED, RSP_NACK, RSP_OTA_NACK], timeout=10.0)
        if result is None:
            print("  Error: No response to OTA_START (is firmware OTA-capable?)")
            return False
        
        seq, type_id, payload = result
        if type_id == RSP_OTA_STARTED:
            d = decode_ota_started(payload)
            if d:
                slot_name = "A" if d["inactive_slot"] == 0 else "B"
                print(f"  ESP32 ready: writing to slot {slot_name} (size: {d['slot_size']} bytes)")
                if total_size > d["slot_size"]:
                    print(f"  Error: Firmware too large for slot!")
                    self._send_frame(cmd_ota_abort(self._next_seq()))
                    return False
                
                # Clear any stale data in buffers (e.g., timeouts from previous sessions)
                time.sleep(0.2)
                self.ser.reset_input_buffer()
                self.rx_buffer.clear()
                self._log("Cleared buffers before chunk transfer")
        else:
            print("  Error: OTA_START rejected")
            return False
        
        # Step 2: Send chunks
        print(f"\n[2/3] Sending firmware chunks...")
        offset = 0
        chunk_count = 0
        start_time = time.time()
        last_progress = -1
        
        while offset < total_size:
            chunk_end = min(offset + CHUNK_SIZE, total_size)
            chunk_data = firmware_data[offset:chunk_end]
            chunk_len = len(chunk_data)
            
            # Send chunk with retry
            success = False
            for retry in range(MAX_RETRIES):
                frame = cmd_ota_chunk(self._next_seq(), offset, chunk_data)
                if not self._send_frame(frame):
                    continue
                
                # Wait for ACK
                result = self._wait_for_response([RSP_OTA_CHUNK, RSP_NACK, RSP_OTA_NACK], timeout=5.0)
                if result is None:
                    self._log(f"  Chunk {chunk_count} timeout, retry {retry + 1}")
                    continue
                
                seq, type_id, payload = result
                if type_id == RSP_OTA_CHUNK:
                    d = decode_ota_chunk_ack(payload)
                    if d:
                        progress = d["progress_pct"]
                        if progress != last_progress:
                            # Progress bar
                            bar_len = 40
                            filled = int(bar_len * progress / 100)
                            bar = "=" * filled + "-" * (bar_len - filled)
                            elapsed = time.time() - start_time
                            speed = d["bytes_written"] / elapsed / 1024 if elapsed > 0 else 0
                            print(f"\r  [{bar}] {progress:3d}% ({d['bytes_written']:,} bytes, {speed:.1f} KB/s)", end="", flush=True)
                            last_progress = progress
                    success = True
                    break
                elif type_id in (RSP_NACK, RSP_OTA_NACK):
                    print(f"\n  Error: Chunk rejected at offset {offset}")
                    self._send_frame(cmd_ota_abort(self._next_seq()))
                    return False
            
            if not success:
                print(f"\n  Error: Failed to send chunk at offset {offset} after {MAX_RETRIES} retries")
                self._send_frame(cmd_ota_abort(self._next_seq()))
                return False
            
            offset = chunk_end
            chunk_count += 1
            time.sleep(CHUNK_DELAY)  # Small delay to prevent buffer overflow
        
        elapsed = time.time() - start_time
        print(f"\n  Sent {chunk_count} chunks in {elapsed:.1f}s ({total_size / elapsed / 1024:.1f} KB/s avg)")
        
        # Step 3: Send OTA_END
        print(f"\n[3/3] Finalizing OTA (verifying checksum)...")
        frame = cmd_ota_end(self._next_seq())
        if not self._send_frame(frame):
            return False
        
        # Wait for ACK_OTA_DONE (may take a few seconds for checksum verification)
        result = self._wait_for_response([RSP_OTA_DONE, RSP_NACK, RSP_OTA_NACK], timeout=30.0)
        if result is None:
            print("  Error: No response to OTA_END")
            return False
        
        seq, type_id, payload = result
        if type_id == RSP_OTA_DONE:
            d = decode_ota_done(payload)
            if d and d["status"] == 0:
                print("  OTA successful! ESP32 is rebooting into new firmware...")
                return True
            else:
                print(f"  OTA failed: status={d['status'] if d else 'unknown'}")
                return False
        elif type_id == RSP_OTA_NACK:
            d = decode_ota_nack(payload)
            if d:
                print(f"  OTA verification failed: {d['error_name']}")
            return False
        else:
            print("  Error: Unexpected response to OTA_END")
            return False


def find_firmware(project_dir: str) -> Optional[str]:
    """Find firmware.bin from PlatformIO build (prefer most recently modified)."""
    candidates = [
        os.path.join(project_dir, ".pio", "build", "esp32dev", "firmware.bin"),
        os.path.join(project_dir, ".pio", "build", "esp32s3_ota", "firmware.bin"),
        os.path.join(project_dir, ".pio", "build", "esp32s3-devkitc-1", "firmware.bin"),
    ]
    # Find the most recently modified firmware
    found = []
    for path in candidates:
        if os.path.exists(path):
            mtime = os.path.getmtime(path)
            found.append((path, mtime))
    
    if not found:
        return None
    
    # Return most recently modified
    found.sort(key=lambda x: x[1], reverse=True)
    return found[0][0]


def main():
    parser = argparse.ArgumentParser(
        description="UART OTA Firmware Upload Tool for ESP32 Pan-Tilt Controller",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python uart_ota.py -p COM9                    # Auto-find firmware
  python uart_ota.py -p COM9 -f firmware.bin    # Specify firmware file
  python uart_ota.py -p COM9 -v                 # Verbose output
  python uart_ota.py -p COM9 --hash sha256      # Use SHA256 instead of CRC32
"""
    )
    parser.add_argument("-p", "--port", required=True, help="Serial port (e.g., COM9, /dev/ttyUSB0)")
    parser.add_argument("-f", "--firmware", help="Firmware binary file path")
    parser.add_argument("-b", "--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--hash", choices=["none", "crc32", "sha256"], default="crc32", help="Hash type (default: crc32)")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    
    args = parser.parse_args()
    
    # Determine firmware path
    if args.firmware:
        firmware_path = args.firmware
    else:
        # Auto-find from PlatformIO build
        project_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        firmware_path = find_firmware(project_dir)
        if not firmware_path:
            print("Error: Could not find firmware.bin in PlatformIO build directories.")
            print("Specify firmware path with -f option or run 'pio run' first.")
            return 1
        print(f"Auto-detected firmware: {firmware_path}")
    
    # Hash type
    hash_type = {"none": OTA_HASH_NONE, "crc32": OTA_HASH_CRC32, "sha256": OTA_HASH_SHA256}[args.hash]
    
    # Create uploader
    uploader = OTAUploader(args.port, args.baud, args.verbose)
    
    if not uploader.connect():
        return 1
    
    try:
        print("\n" + "=" * 60)
        print("  UART OTA Firmware Upload")
        print("=" * 60 + "\n")
        
        success = uploader.upload(firmware_path, hash_type)
        
        if success:
            print("\n" + "=" * 60)
            print("  OTA COMPLETE - Firmware updated successfully!")
            print("=" * 60)
            return 0
        else:
            print("\n" + "=" * 60)
            print("  OTA FAILED - See errors above")
            print("=" * 60)
            return 1
    
    finally:
        uploader.disconnect()


if __name__ == "__main__":
    sys.exit(main())

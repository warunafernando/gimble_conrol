# UART OTA Build and Update Guide

This document explains how to build firmware with OTA support and how to perform UART OTA updates for the ESP32 Pan-Tilt Controller. Use this guide when you need to build or update the firmware in the future.

---

## Overview

UART OTA allows you to update the ESP32 firmware over the existing USB serial connection **without** manually entering bootloader mode (no BOOT+RESET button sequence required). The running firmware receives new firmware chunks and writes them to the inactive OTA partition (A/B scheme), verifies the checksum, then commits and reboots into the new image.

---

## Prerequisites

- **Hardware:** Waveshare ESP32 "General Driver for Robots" board (or compatible)
- **Software:** Python 3, pyserial, PlatformIO
- **Initial flash:** The ESP32 must have OTA-capable firmware at least once (via manual bootloader)

---

## Part 1: Building Firmware with OTA Support

### 1.1 Ensure OTA Partitions

The project uses an A/B partition scheme. Check that `partitions.csv` or `partitions_4mb.csv` includes:
- `ota_0` and `ota_1` (app slots)
- `otadata` (OTA state)

### 1.2 Build the Firmware

From the project root:

```bash
cd e:\Arduino_Controller\pan_tilt_serial_project
pio run -e esp32dev
```

This produces:
- `.pio/build/esp32dev/firmware.bin` – main firmware binary
- `.pio/build/esp32dev/bootloader.bin`
- `.pio/build/esp32dev/partitions.bin`

The OTA tool uses `firmware.bin` from the build output.

### 1.3 Clean Build (if needed)

If you suspect stale artifacts:

```bash
pio run -e esp32dev -t clean
pio run -e esp32dev
```

---

## Part 2: Performing UART OTA Updates

### 2.1 Using the OTA Tool Directly

**Requirements:** ESP32 must already be running OTA-capable firmware.

```bash
# Auto-detect firmware, use default COM port
python tools/uart_ota.py -p COM9

# With specific firmware file
python tools/uart_ota.py -p COM9 -f .pio/build/esp32dev/firmware.bin

# Verbose output for debugging
python tools/uart_ota.py -p COM9 -v

# Different baud rate (default is 921600)
python tools/uart_ota.py -p COM9 -b 115200

# Use SHA256 instead of CRC32 for verification
python tools/uart_ota.py -p COM9 --hash sha256
```

### 2.2 Using ota_load.bat (recommended – no timeout)

Double-click `tools\ota_load.bat` to run OTA in a separate window. Best when running from IDE or automated tools, since it avoids shell timeout. Builds firmware first if needed.

```bash
# Double-click in Explorer, or from command line:
tools\ota_load.bat
tools\ota_load.bat COM9   # specify port
```

### 2.3 Using load_new_fw.bat (OTA + manual fallback)

`load_new_fw.bat` tries UART OTA first, then falls back to manual bootloader if OTA is not available or fails:

```bash
# Default port (COM9)
tools\load_new_fw.bat

# Specific port
tools\load_new_fw.bat COM9

# Skip OTA, go straight to manual bootloader
tools\load_new_fw.bat COM9 --manual
```

### 2.4 Expected OTA Flow

1. **Connect** – Tool opens serial port at 921600 baud
2. **OTA_START** – Sends size, hash type, and expected CRC32
3. **Chunks** – Sends firmware in ~240-byte chunks with progress bar
4. **OTA_END** – ESP32 verifies checksum and commits
5. **Reboot** – ESP32 reboots into new firmware

Typical transfer time: ~40–60 seconds for ~300 KB firmware at 921600 baud.

---

## Part 3: Manual Bootloader Fallback

If UART OTA fails (e.g., first-time flash or OTA not supported), use the manual bootloader method.

### 3.1 Put ESP32 in Bootloader Mode

1. Hold the **BOOT** button
2. Press and release the **RESET** button
3. Release **BOOT**

### 3.2 Upload via esptool

```bash
# Rapid retry (run immediately after entering bootloader mode)
python -m esptool --chip esp32 --port COM9 --baud 460800 --before no_reset --after hard_reset write_flash 0x1000 .pio/build/esp32dev/bootloader.bin 0x8000 .pio/build/esp32dev/partitions.bin 0x10000 .pio/build/esp32dev/firmware.bin
```

Or use the Waveshare uploader:

```bash
python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev --manual
```

---

## Part 4: Troubleshooting

| Problem | Possible cause | Solution |
|---------|----------------|----------|
| "No response to OTA_START" | ESP32 not running OTA firmware, or wrong port | First flash via manual bootloader; confirm COM port |
| "CHECKSUM_FAIL" | CRC mismatch | Ensure firmware built clean; do not modify `.bin`; check CRC table in firmware |
| "Chunk rejected" | State/timeout issue | Power cycle ESP32; try again after full boot |
| OTA succeeds but device hangs | Bad image or boot config | Re-flash via manual bootloader |
| Serial port not found | USB disconnected or wrong port | Check Device Manager; use correct COM port |
| OTA times out when run from IDE | Shell timeout (~60–90 s) | Use `tools\ota_load.bat` (double-click) or run OTA in background |

### Verify Serial Connection

```bash
python -c "
import serial
s = serial.Serial('COM9', 921600)
s.reset_input_buffer()
s.close()
print('Port OK')
"
```

---

## Part 5: File Reference

| File | Purpose |
|------|---------|
| `tools/uart_ota.py` | Main UART OTA upload tool |
| `tools/ota_load.bat` | Double-click OTA (no timeout); builds if needed |
| `tools/uart_ota.bat` | Convenience wrapper for OTA |
| `tools/load_new_fw.bat` | OTA first, then manual bootloader fallback |
| `tools/waveshare_uploader.py` | Manual bootloader upload helper |
| `UART_OTA_PLAN.md` | Design and validation plan |
| `GIMBAL_PROTOCOL.md` | OTA command/response definitions |

---

## Quick Reference: Common Commands

```bash
# Build firmware
pio run -e esp32dev

# UART OTA update (ESP32 must already run OTA-capable firmware)
python tools/uart_ota.py -p COM9

# Full workflow (OTA first, manual fallback)
tools\load_new_fw.bat COM9

# Manual bootloader upload (hold BOOT, press RESET, then run)
python -m esptool --chip esp32 --port COM9 --baud 460800 --before no_reset --after hard_reset write_flash 0x1000 .pio/build/esp32dev/bootloader.bin 0x8000 .pio/build/esp32dev/partitions.bin 0x10000 .pio/build/esp32dev/firmware.bin
```

---

---

## Part 6: Firmware Version and Slot Switching

### Get FW Info (GET_FW_INFO, type 610)

Returns active slot (A=0, B=1) and version strings for both A and B images:

```python
# Python (protocol.py)
from protocol import cmd_get_fw_info, decode_fw_info
frame = cmd_get_fw_info(seq)
# Send frame, receive FW_INFO (type 2610)
# decode_fw_info(payload) → {"active_slot": 0, "version_a": "1.0.0", "version_b": "---"}
```

### Switch Active FW (SWITCH_FW, type 611)

Switch boot partition and reboot:

```python
# Switch to slot A (ota_0)
frame = cmd_switch_fw(seq, 0)

# Switch to slot B (ota_1)
frame = cmd_switch_fw(seq, 1)
```

### Version in Build

Set `APP_VERSION` in `platformio.ini` per environment:

```ini
build_flags = -D APP_VERSION=\"1.0.0\"
```

Or edit the default in `pan_tilt_serial_project.ino` if `APP_VERSION` is not defined.

---

*Document version: 1.0 | Last updated: Jan 2025*

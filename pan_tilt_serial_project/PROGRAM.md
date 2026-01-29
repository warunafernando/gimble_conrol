# ESP32 Upload Tool Development - Program Documentation

## Problem Statement

PlatformIO's default `esptool` upload method fails with the Waveshare General Driver for Robots board:
```
Failed to connect to ESP32: Wrong boot mode detected (0x13)! The chip needs to be in download mode.
```

### Root Cause

1. **Auto-download circuit timing**: The Waveshare board has an automatic download circuit that uses DTR/RTS signals, but needs ~100-500ms to activate
2. **Strict boot mode checking**: esptool v4.9.0+ performs strict boot mode validation that checks too early
3. **DTR/RTS handling**: The CP2102 driver on this system doesn't reliably handle DTR/RTS signals for automatic download mode entry

## Solution: Waveshare-Compatible Uploader Tool

Created a Python tool (`tools/waveshare_uploader.py`) that mimics the successful `flash_esp32_cli.py` method.

### Key Features

1. **Manual Download Mode**: Uses `--manual` flag to wait for user to put ESP32 in download mode
2. **No DTR/RTS dependency**: Uses `--before no_reset` so esptool doesn't try automatic reset
3. **Command-line esptool**: Uses esptool via subprocess (not Python API) for better reliability
4. **PlatformIO integration**: Auto-detects build files from `.pio/build/esp32dev/`
5. **Multi-file upload**: Uploads bootloader, partitions, and firmware in one command

## How It Works

### Manual Mode Workflow

1. **User puts ESP32 in download mode**:
   - Hold BOOT button
   - Press and release RESET button
   - Release BOOT button

2. **Tool waits 5 seconds** (countdown displayed)

3. **Tool runs esptool command**:
   ```bash
   python -m esptool --chip esp32 --port COM9 --baud 115200 \
     --before no_reset --after hard_reset \
     write_flash -z --flash_mode dio --flash_freq 80m \
     0x1000 bootloader.bin \
     0x8000 partitions.bin \
     0x10000 firmware.bin
   ```

4. **esptool connects** to ESP32 already in download mode (no reset needed)

5. **Uploads all files** and performs hard reset

## Files Created

### Main Tool
- **`tools/waveshare_uploader.py`**: Main uploader tool (368 lines)
  - Manual mode support
  - Auto-detect PlatformIO build files
  - Uses esptool command line
  - Error handling and retries

### Convenience Scripts
- **`tools/upload.bat`**: Windows batch script for quick upload
- **`tools/platformio_upload.py`**: PlatformIO integration script

### Documentation
- **`tools/README.md`**: Complete usage guide
- **`tools/INSTALL.md`**: Installation instructions
- **`tools/QUICK_START.md`**: Quick reference guide
- **`tools/requirements.txt`**: Python dependencies
- **`TOOL_CREATED.md`**: Tool creation summary
- **`UPLOAD_SOLUTION.md`**: Problem analysis and solution
- **`UPLOAD_FIX.md`**: Troubleshooting guide

## Usage

### Basic Upload (Manual Mode - Recommended)

```bash
python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev --manual
```

**Steps:**
1. Put ESP32 in download mode (hold BOOT, press RESET, release BOOT)
2. Run command immediately
3. Tool waits 5 seconds (countdown)
4. Upload proceeds automatically

### PlatformIO Integration

The tool is integrated into `platformio.ini`:
```ini
extra_scripts = 
    extra_scripts/upload_fixed.py
    tools/platformio_upload.py
```

Use normal PlatformIO upload:
```bash
pio run -t upload --upload-port COM9
```

### Quick Upload Script (Windows)

```bash
tools\upload.bat
```

## Successful Test Results

### Test 1: COM6
- **Date**: 2026-01-26
- **Result**: ✅ SUCCESS
- **Chip**: ESP32-D0WD-V3 (revision v3.1)
- **MAC**: cc:7b:5c:1f:14:a0
- **Files uploaded**:
  - Bootloader: 17,536 bytes at 0x1000
  - Partitions: 3,072 bytes at 0x8000
  - Firmware: 315,264 bytes at 0x10000

### Test 2: COM9
- **Date**: 2026-01-26
- **Result**: ✅ SUCCESS
- **Chip**: ESP32-D0WD-V3 (revision v3.1)
- **MAC**: cc:7b:5c:1f:07:68
- **Files uploaded**: Same as COM6

## Technical Details

### Why Manual Mode Works

1. **No timing dependency**: User puts ESP32 in download mode before tool connects
2. **No DTR/RTS needed**: Uses `--before no_reset` flag
3. **Reliable connection**: esptool connects to ESP32 already in download mode
4. **Proven method**: Same approach as `flash_esp32_cli.py` which worked successfully yesterday

### Comparison with Other Methods

| Method | Works? | Why |
|--------|--------|-----|
| PlatformIO default upload | ❌ | Strict boot mode check, timing issues |
| esptool with auto-reset | ❌ | DTR/RTS doesn't work reliably |
| Waveshare GUI tool | ✅ | Designed for this hardware |
| **This tool (manual mode)** | ✅ | Matches successful CLI method |

## Dependencies

```bash
pip install esptool pyserial
```

Or use requirements file:
```bash
pip install -r tools/requirements.txt
```

## Key Learnings

1. **Manual download mode entry is most reliable** for this hardware
2. **Using esptool command line** (subprocess) works better than Python API for this use case
3. **5-second countdown** gives user enough time to put ESP32 in download mode
4. **`--before no_reset`** is critical - tells esptool not to try automatic reset
5. **PlatformIO integration** makes it seamless to use

## Future Improvements

Potential enhancements:
- Auto-detect COM port
- Support for multiple devices simultaneously
- Progress bar during upload
- Automatic retry with manual mode prompt
- GUI version for easier use

## References

- **Successful method**: `flash_esp32_cli.py` (root directory)
- **Original guide**: `FIRMWARE_UPLOAD_GUIDE.md`
- **Waveshare tool guide**: `WAVESHARE_FLASH_TOOL_GUIDE.md`
- **Problem analysis**: `UPLOAD_SOLUTION.md`

## Notes

- Tool works on both COM6 and COM9
- Requires manual download mode entry (most reliable)
- Uses same approach as proven successful method
- Fully integrated with PlatformIO workflow
- Cross-platform (Windows tested, should work on Linux/Mac)

## GUI Test Utility

A comprehensive Windows GUI application (`tools/pan_tilt_test_gui.py`) for testing all board functions:

### Features
- **Pan/Tilt Control**: Absolute, relative, single-axis, lock/unlock
- **IMU Data Display**: Roll, pitch, yaw, accelerometer, gyroscope, magnetometer, temperature
- **Power Monitoring**: Voltage, current, power consumption, overflow detection
- **Servo Feedback**: Position, speed, load, voltage, temperature, mode for both servos
- **System Control**: Heartbeat timeout, feedback interval, enable/disable
- **Quick Tests**: Sensor tests, sweep patterns, figure-8 motion
- **Log Console**: Real-time communication log with save capability

### Usage
```bash
python tools/pan_tilt_test_gui.py
```

Or use the launcher:
```bash
tools\run_test_gui.bat
```

### Requirements
- Python 3.7+ with tkinter (usually included)
- pyserial: `pip install pyserial`

See `tools/test_gui_README.md` for complete documentation.

---

**Created**: 2026-01-26  
**Status**: ✅ Working and tested successfully  
**Tested on**: COM6 and COM9  
**GUI Tool**: ✅ Created and ready for use

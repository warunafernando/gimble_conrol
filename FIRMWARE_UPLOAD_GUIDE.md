# Firmware Upload Guide - Waveshare Pan-Tilt ESP32

**Last successful upload:** Using `flash_esp32_cli.py` with **manual download mode** (no DTR/RTS).

## Quick Reference

**Command that works** (run from `E:\Arduino_Controller`):

For **pan_tilt_serial_project** (JSON serial / test GUI):
```bash
python flash_esp32_cli.py --port COM9 --platformio --manual --project-dir pan_tilt_serial_project
```

For **pan_tilt_base_v0.9** (original firmware):
```bash
python flash_esp32_cli.py --port COM9 --platformio --manual --project-dir pan_tilt_base_v0.9
```

## Step-by-Step Procedure

### 1. Compile the Firmware

First, make sure the firmware is compiled (from `E:\Arduino_Controller`):

```bash
pio run -d pan_tilt_serial_project
```

Or for the original firmware: `pio run -d pan_tilt_base_v0.9`

Wait for compilation to complete successfully.

### 2. Connect Hardware

1. **Connect USB cable** to the **middle-side USB port** (labeled "USB", port #9) on the General Driver for Robots board
   - ⚠️ **NOT** the LIDAR Type-C port
2. Verify COM port appears in Device Manager (should show as "Silicon Labs CP210x USB to UART Bridge")

### 3. Put ESP32 in Download Mode (MANUAL)

**This is the critical step that makes it work:**

1. **Hold the BOOT button** (Download button, button #6 on the board)
2. **Press and release the RESET button** (button #5 on the board) - while still holding BOOT
3. **Release the BOOT button**
4. ESP32 is now in download mode

**Important:** Keep the ESP32 in this state - don't release buttons too early!

### 4. Run Flash Command

**Immediately after putting ESP32 in download mode**, run (from `E:\Arduino_Controller`):

```bash
python flash_esp32_cli.py --port COM9 --platformio --manual --project-dir pan_tilt_serial_project
```

The `--manual` flag tells the tool:
- Not to try automatic DTR/RTS reset (which doesn't work with this CP2102 driver)
- To wait 3 seconds for you to put ESP32 in download mode
- To connect to ESP32 that's already in download mode

### 5. Wait for Completion

The tool will:
- Wait 3 seconds (giving you time to put ESP32 in download mode)
- Connect to ESP32
- Flash all segments:
  - Bootloader (0x1000)
  - Partitions (0x8000)
  - Boot App0 (0xe000)
  - Firmware (0x10000)
- Verify with hash checks
- Perform hard reset

**Success message:** `[SUCCESS] Flash completed successfully!`

### 6. Power On

After successful flash:
1. Disconnect USB cable
2. Connect **12V 5A power supply** to the XH2.54 power port
3. Turn on the power switch on the driver board
4. Check OLED screen - should display "PT CAM Version:0.9"

## Why Manual Mode is Required

**Problem:** The CP2102 USB-to-UART driver on this system doesn't properly handle DTR/RTS signals needed for automatic download mode entry.

**Solution:** Manual download mode entry:
- You manually put ESP32 in download mode using buttons
- The tool connects to ESP32 that's already in download mode
- No DTR/RTS signals needed

## Alternative Methods (If Manual Mode Fails)

### Option 1: Waveshare GUI Tool

Download from: https://files.waveshare.com/wiki/2-Axis-Pan-Tilt-Camera-Module/2-Axis_Pan-Tilt_Camera_Module_FACTORY.zip

1. Extract and run `flash_download_tool_3.9.5.exe`
2. Select Chip Type: **ESP32**, WorkMode: **Factory**
3. Select COM port (COM9) and baud rate (921600 max)
4. Put ESP32 in download mode manually (same procedure as above)
5. Click **START**

### Option 2: PlatformIO Upload (Usually Doesn't Work)

```bash
pio run -d E:\Arduino_Controller\pan_tilt_base_v0.9 --target upload --upload-port COM9
```

**Note:** This usually fails because it relies on DTR/RTS signals that don't work with this CP2102 driver.

## Troubleshooting

### "Connection failed" or "Wrong boot mode detected"

**Solution:** Make sure ESP32 is in download mode BEFORE running the command:
1. Hold BOOT
2. Press and release RESET
3. Release BOOT
4. **Then immediately** run the flash command

### "No serial data received"

**Possible causes:**
- ESP32 exited download mode before connection
- Wrong COM port
- USB cable issue
- Another program using COM9

**Solutions:**
- Try again with faster timing (put in download mode, then immediately run command)
- Check Device Manager for correct COM port
- Close any serial monitor or other programs using COM9
- Try different USB cable/port

### "Build directory not found"

**Solution:** Compile firmware first:
```bash
pio run -d E:\Arduino_Controller\pan_tilt_base_v0.9
```

### COM Port Not Showing

**Solution:** Install CP2102 driver:
- Download: https://files.waveshare.com/upload/d/d6/CP210x_USB_TO_UART.zip
- Extract and run `CP210xVCPInstaller_x64.exe`
- Unplug and replug USB cable

## Command Options

| Option | Description |
|--------|-------------|
| `--port COM9` | COM port (required) |
| `--platformio` | Auto-detect firmware from PlatformIO build |
| `--manual` | **REQUIRED** - Manual download mode (no DTR/RTS) |
| `--project-dir DIR` | PlatformIO project directory |
| `--baud RATE` | Baud rate (default: 115200, max: 921600) |
| `--erase` | Erase flash before writing (optional) |

## Complete Workflow Example

**pan_tilt_serial_project** (this project):

```bash
cd E:\Arduino_Controller

# 1. Compile firmware
pio run -d pan_tilt_serial_project

# 2. Put ESP32 in download mode:
#    - Hold BOOT
#    - Press and release RESET
#    - Release BOOT

# 3. Flash firmware (run immediately after step 2)
python flash_esp32_cli.py --port COM9 --platformio --manual --project-dir pan_tilt_serial_project

# 4. After success: disconnect USB, power as needed
```

**pan_tilt_base_v0.9** (original):

```bash
cd E:\Arduino_Controller
pio run -d pan_tilt_base_v0.9
# Put ESP32 in download mode (BOOT + RESET), then:
python flash_esp32_cli.py --port COM9 --platformio --manual --project-dir pan_tilt_base_v0.9
```

## Key Success Factors

1. ✅ **Manual download mode entry** - Put ESP32 in download mode BEFORE running command
2. ✅ **Timing** - Run flash command immediately after putting ESP32 in download mode
3. ✅ **Correct USB port** - Use middle-side USB port (labeled "USB"), not LIDAR port
4. ✅ **Using `--manual` flag** - Tells tool not to try automatic reset
5. ✅ **3-second delay** - Tool waits 3 seconds, giving you time to put ESP32 in download mode

## Notes

- The board has an "Automatic download circuit" (#27) but it doesn't work reliably with this CP2102 driver
- Manual mode is more reliable for this hardware setup
- The CLI tool (`flash_esp32_cli.py`) works better than PlatformIO's built-in upload for this board
- Firmware files are auto-detected from PlatformIO build directory - no need to specify paths manually

## Files Created

- `flash_esp32_cli.py` - Command-line flash tool (works!)
- `FIRMWARE_UPLOAD_GUIDE.md` - This guide
- `WAVESHARE_FLASH_TOOL_GUIDE.md` - Waveshare GUI tool guide
- `ESP32_FLASH_TOOL_GUIDE.md` - Generic Espressif tool guide

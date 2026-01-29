# ESP32 Flash Tool - Command Line Interface

A command-line alternative to the Waveshare GUI flash tool. This tool can flash firmware without needing the GUI interface.

## Features

- ✅ **Auto-detects PlatformIO firmware files** - No need to specify paths manually
- ✅ **Works with manual download mode** - No DTR/RTS signals required
- ✅ **Uses PlatformIO's esptool** - No need to install esptool separately
- ✅ **Retry logic** - Automatically retries failed connections
- ✅ **Clear feedback** - Shows progress and status like GUI tool

## Installation

No installation needed! Just requires:
- Python 3 (already installed)
- PlatformIO (for PlatformIO firmware flashing)
- Or esptool module (for custom firmware)

## Usage

### Flash PlatformIO Firmware (Recommended)

```bash
# Basic usage - auto-detects firmware files
python flash_esp32_cli.py --port COM9 --platformio

# With manual download mode (if DTR/RTS doesn't work)
python flash_esp32_cli.py --port COM9 --platformio --manual

# With flash erase and higher baud rate
python flash_esp32_cli.py --port COM9 --platformio --erase --baud 921600

# Custom project directory
python flash_esp32_cli.py --port COM9 --platformio --project-dir my_project
```

### Flash Custom Firmware

```bash
# Flash with custom segments
python flash_esp32_cli.py --port COM9 \
  --segment 0x1000:bootloader.bin \
  --segment 0x8000:partitions.bin \
  --segment 0xe000:boot_app0.bin \
  --segment 0x10000:firmware.bin

# With manual mode and erase
python flash_esp32_cli.py --port COM9 \
  --segment 0x1000:bootloader.bin \
  --segment 0x10000:app.bin \
  --manual --erase
```

## Command Options

| Option | Description |
|--------|-------------|
| `--port` | COM port (required, e.g. COM9) |
| `--platformio` | Flash from PlatformIO build directory |
| `--segment OFFSET:FILE` | Flash custom segment (can repeat) |
| `--baud RATE` | Baud rate (default: 115200, max: 921600) |
| `--erase` | Erase flash before writing |
| `--manual` | Manual download mode (no DTR/RTS) |
| `--project-dir DIR` | PlatformIO project directory (default: pan_tilt_base_v0.9) |

## Manual Download Mode

If your CP2102 driver doesn't handle DTR/RTS signals properly, use `--manual`:

1. **Put ESP32 in download mode:**
   - Hold **BOOT** button
   - Press and release **RESET** button
   - Release **BOOT** button

2. **Run the command with `--manual` flag:**
   ```bash
   python flash_esp32_cli.py --port COM9 --platformio --manual
   ```

3. The tool will wait 3 seconds for you to put it in download mode, then flash.

## Comparison with GUI Tool

| Feature | GUI Tool | This CLI Tool |
|---------|---------|---------------|
| Pre-configured firmware | ✅ (Factory mode) | ✅ (PlatformIO auto-detect) |
| Custom firmware | ❌ | ✅ |
| Manual download mode | ✅ | ✅ |
| Command-line/scriptable | ❌ | ✅ |
| Retry logic | Limited | ✅ (3 retries) |
| Progress feedback | ✅ (GUI) | ✅ (Text) |

## Troubleshooting

**"Build directory not found"**
- Compile firmware first: `pio run -d pan_tilt_base_v0.9`

**"Connection failed"**
- Try `--manual` mode
- Check COM port is correct
- Try lower baud rate (115200)
- Make sure no other program is using the port

**"File not found"**
- For PlatformIO: Make sure you've compiled the firmware
- For custom: Check file paths are correct

## Examples

### Complete Workflow

```bash
# 1. Compile firmware
pio run -d pan_tilt_base_v0.9

# 2. Flash firmware (automatic mode)
python flash_esp32_cli.py --port COM9 --platformio

# Or flash with manual mode (if automatic fails)
python flash_esp32_cli.py --port COM9 --platformio --manual
```

### Fast Flash (High Baud Rate)

```bash
python flash_esp32_cli.py --port COM9 --platformio --baud 921600
```

### Clean Flash (Erase First)

```bash
python flash_esp32_cli.py --port COM9 --platformio --erase
```

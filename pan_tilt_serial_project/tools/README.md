# Tools

## Web GUI

To run the gimbal control GUI: **`start_gui.bat`** (starts backend and opens http://localhost:5000). See `../backend/README.md` for backend details.

---

## Waveshare-Compatible ESP32 Uploader

A Python tool that mimics the Waveshare Flash Download Tool behavior for reliable ESP32 firmware uploads on the General Driver for Robots board.

## Why This Tool?

The standard `esptool` fails with the Waveshare board due to strict boot mode checking. This tool:
- ✅ Properly activates the auto-download circuit with correct DTR/RTS timing
- ✅ Waits for the circuit to stabilize before connecting
- ✅ Uses esptool's Python API directly (bypasses strict command-line checks)
- ✅ Works reliably with the Waveshare General Driver for Robots board

## Installation

1. **Install Python 3.7+** (if not already installed)

2. **Install required packages:**
   ```bash
   pip install esptool pyserial
   ```

## Usage

### Quick Upload (PlatformIO Project)

From the project root:
```bash
# Windows
tools\upload.bat

# Linux/Mac
python tools/waveshare_uploader.py -p /dev/ttyUSB0 --build-dir .pio/build/esp32dev
```

### Manual Upload

**Upload firmware only:**
```bash
python tools/waveshare_uploader.py -p COM9 firmware.bin
```

**Upload all components:**
```bash
python tools/waveshare_uploader.py -p COM9 \
    --bootloader bootloader.bin \
    --partitions partitions.bin \
    --firmware firmware.bin
```

**Using PlatformIO build directory:**
```bash
python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev
```

### Command Line Options

```
-p, --port PORT          Serial port (required, e.g., COM9 or /dev/ttyUSB0)
-b, --baud BAUD           Baud rate (default: 115200)
--firmware FILE           Firmware binary file
--firmware-addr ADDR      Firmware address (default: 0x10000)
--bootloader FILE         Bootloader binary file
--bootloader-addr ADDR    Bootloader address (default: 0x1000)
--partitions FILE         Partitions binary file
--partitions-addr ADDR    Partitions address (default: 0x8000)
--build-dir DIR           PlatformIO build directory (auto-detects files)
```

## How It Works

1. **Auto-Download Circuit Activation:**
   - Sends proper DTR/RTS sequence to trigger the auto-download circuit
   - Waits 600ms for the circuit to stabilize (critical timing!)

2. **Connection:**
   - Connects to ESP32 using esptool's Python API
   - Uses `no_reset` mode (we already handled reset via DTR/RTS)
   - Retries connection if needed

3. **Upload:**
   - Uploads bootloader, partitions, and firmware in sequence
   - Uses esptool's flash_file method for reliable flashing

4. **Reset:**
   - Performs hard reset after successful upload

## Troubleshooting

**"Could not open serial port":**
- Check COM port is correct
- Make sure no other program is using the port
- Close serial monitors/other tools

**"Failed to connect":**
- Try manually putting ESP32 in download mode:
  - Hold BOOT button
  - Press and release RESET button
  - Release BOOT button
- Check USB cable connection
- Try different COM port

**"Upload failed":**
- Verify firmware file exists and is valid
- Check ESP32 flash size matches your firmware
- Try lower baud rate: `-b 115200`

## Comparison with Waveshare Tool

| Feature | Waveshare Tool | This Tool |
|---------|---------------|-----------|
| Auto-download circuit | ✅ Works | ✅ Works |
| Boot mode check | ❌ Not strict | ✅ Bypassed |
| DTR/RTS timing | ✅ Optimized | ✅ Optimized |
| PlatformIO integration | ❌ Manual | ✅ Automatic |
| Command line | ❌ GUI only | ✅ CLI + scripts |
| Cross-platform | ❌ Windows | ✅ All platforms |

## Integration with PlatformIO

You can integrate this tool into PlatformIO by modifying `platformio.ini`:

```ini
[env:esp32dev]
# ... other settings ...

extra_scripts = tools/platformio_upload.py
```

Then create `tools/platformio_upload.py`:
```python
Import("env")
import subprocess
import sys

def upload_with_waveshare(source, target, env):
    firmware = str(source[0])
    port = env.get("UPLOAD_PORT", "COM9")
    build_dir = env.get("BUILD_DIR")
    
    cmd = [
        sys.executable,
        "tools/waveshare_uploader.py",
        "-p", port,
        "--build-dir", build_dir
    ]
    
    env.Execute(" ".join(cmd))

env.Replace(UPLOADCMD=upload_with_waveshare)
```

## License

This tool is provided as-is for use with the Waveshare General Driver for Robots board.

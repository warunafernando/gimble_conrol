# Quick Start Guide

## Start Web GUI

Double-click `start_gui.bat` or run:
```bash
tools\start_gui.bat
```
This starts the backend and opens http://localhost:5000 in your browser. The backend auto-connects to the gimbal (model 99) on any COM port. Use **Rescan and connect** if the gimbal was unplugged.

## Common Issues

### Port Already in Use

If you see:
```
ERROR: Serial port COM9 is already in use!
```

**Solution:**
1. Close any serial monitors (Arduino IDE, PlatformIO Monitor, etc.)
2. Close any terminal programs using COM9
3. Wait a few seconds and try again
4. If still locked, unplug and replug the USB cable

### Connection Hangs

If the tool seems to hang during connection:

1. **Check the port is correct:**
   - Windows: Check Device Manager for COM port number
   - Make sure you're using the ESP32 port, not the LIDAR port

2. **Try manual boot mode:**
   - Hold BOOT button
   - Press and release RESET button
   - Release BOOT button
   - Run the upload tool immediately

3. **Check USB cable:**
   - Use a data cable (not charge-only)
   - Try a different USB port

## Usage

### UART OTA (recommended – no bootloader needed)

Double-click `ota_load.bat` or run:
```bash
python tools/uart_ota.py -p COM9
```
See `docs/UART_OTA_LOAD_GUIDE.md` for details. Use `ota_load.bat` when running from IDE to avoid timeout.

### Manual Upload (bootloader required)
```bash
python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev
```

### PlatformIO Integration
```bash
pio run -t upload --upload-port COM9
```

The tool will automatically:
- Activate auto-download circuit
- Connect to ESP32
- Upload bootloader, partitions, and firmware
- Reset ESP32

## Troubleshooting

**"Port not found"**
- Check USB cable connection
- Install CP2102 drivers if needed
- Check Device Manager for COM port

**"Failed to connect"**
- Try manual boot mode entry
- Check baud rate (default 115200)
- Try different USB port

**"Upload failed"**
- Check firmware file exists
- Verify flash size matches
- Try lower baud rate: `-b 115200`

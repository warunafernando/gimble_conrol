# Project Instructions

## Firmware Loading: Use UART OTA

When changing ESP32 firmware and loading it to the device, **always use UART OTA** (not manual bootloader).

### Default Load Method

When running via Shell, run OTA in **background** to avoid timeout.

```bash
# Preferred method
python tools/uart_ota.py -p <PORT>

# Or use the batch script (Windows)
tools/ota_load.bat <PORT>

# OTA first, manual fallback if needed
tools/load_new_fw.bat <PORT>
```

### Do NOT Use Manual Bootloader

Unless OTA fails or the device has never had OTA firmware, do **not** use:
- `waveshare_uploader.py --manual`
- `esptool write_flash` directly

### Fallback

Use manual bootloader only when OTA fails (e.g., first-time flash). See `docs/UART_OTA_LOAD_GUIDE.md`.

---

## Project Structure

- `pan_tilt_serial_project/` - Main firmware and tools
  - `*.ino, *.cpp, *.h` - ESP32 firmware (PlatformIO)
  - `backend/` - Python Flask-SocketIO Web GUI
  - `tools/` - OTA upload, test GUI utilities
  - `docs/` - Setup and usage guides
- `SERIAL_PAN_TILT_SPEC.md` - Protocol specification
- `ST3215_MEMORY_REGISTER_MAP.md` - Servo register reference

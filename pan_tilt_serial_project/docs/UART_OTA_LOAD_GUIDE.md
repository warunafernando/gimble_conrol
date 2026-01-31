# How to Load Firmware via UART OTA

A short guide for loading (uploading) firmware to the ESP32 using UART OTA in the future.

---

## Default Method

**When changing ESP32 firmware, always use UART OTA** to load it. Do not use manual bootloader (BOOT+RESET) unless OTA fails.

---

## Requirements

- ESP32 already running OTA-capable firmware (first time must use manual bootloader)
- USB connected, correct COM port (e.g., COM9)
- Firmware built: `pio run -e esp32dev` (creates `.pio/build/esp32dev/firmware.bin`)

---

## Steps

### 1. Build (if needed)

```bash
cd e:\Arduino_Controller\pan_tilt_serial_project
pio run -e esp32dev
```

### 2. Run OTA Load

**Option A – Double-click (recommended, no timeout)**

Double-click `tools\ota_load.bat` in Explorer. It opens a window, runs OTA, and waits for completion. Best when running from IDE or automated tools to avoid timeout.

**Option B – OTA tool only**

```bash
python tools/uart_ota.py -p COM9
```

**Option C – OTA first, then manual bootloader if needed**

```bash
tools\load_new_fw.bat COM9
```

### 3. Wait for completion

- Transfer takes ~40–60 seconds
- ESP32 will reboot into the new firmware
- Success message: `OTA COMPLETE - Firmware updated successfully!`

---

## If OTA Fails

Use manual bootloader:

1. Hold **BOOT**
2. Press and release **RESET**
3. Release **BOOT**
4. Run:

```bash
python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev --manual
```

Or:

```bash
python -m esptool --chip esp32 --port COM9 --baud 460800 --before no_reset --after hard_reset write_flash 0x1000 .pio/build/esp32dev/bootloader.bin 0x8000 .pio/build/esp32dev/partitions.bin 0x10000 .pio/build/esp32dev/firmware.bin
```

---

## Quick Reference

| Action        | Command                                           |
|---------------|---------------------------------------------------|
| Double-click  | `tools\ota_load.bat` (no timeout)                 |
| Load via OTA  | `python tools/uart_ota.py -p COM9`                |
| OTA + fallback| `tools\load_new_fw.bat COM9`                      |
| Verbose       | `python tools/uart_ota.py -p COM9 -v`             |

---

## Avoiding Timeout (IDE / Automated Tools)

When running OTA from Cursor IDE or other automated tools, the shell may timeout (~60–90 s) before the OTA completes (~45 s). To avoid this:

- **Use `tools\ota_load.bat`** – Double-click to run; it opens a separate window and completes without timeout.
- **Run in background** – If using Shell, run OTA with `is_background=true` so it is not killed by the timeout.

---

*For full details, see `UART_OTA_BUILD_GUIDE.md`*

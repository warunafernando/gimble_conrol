# Flash ESP32 on COM9

## Recommended: manual download mode (10 s countdown)

1. **Close** any app using COM9 (backend, serial monitor). Keep USB plugged in.
2. **Run** (from repo root or `pan_tilt_serial_project`):
   ```bash
   tools\load_new_fw.bat
   ```
   Or with another port: `tools\load_new_fw.bat COM6`
3. **During the 10-second countdown**: put ESP32 in download mode:
   - **Hold** **BOOT**, **press and release** **RESET**, then **release** **BOOT**.
   - Do **not** unplug USB.
4. After a successful flash, the board resets and runs the new firmware.

## If auto-upload fails (direct esptool)

1. **Put the ESP32 in download mode** (do this first):
   - **Hold** **BOOT**, **press and release** **RESET**, **release** **BOOT**.
2. **Run the upload immediately** (from `pan_tilt_serial_project`):
   ```bash
   python -m esptool --port COM9 --baud 115200 --before no_reset --after hard_reset --connect-attempts 15 write_flash 0x1000 .pio\build\esp32dev\bootloader.bin 0x8000 .pio\build\esp32dev\partitions.bin 0x10000 .pio\build\esp32dev\firmware.bin
   ```
   Or build first: `pio run` then the command above.
3. After a successful flash, run the auto-tester from `tester/`: `python run_tests.py COM9`.

## Normal upload (if your board enters download mode via DTR/RTS)

```bash
cd pan_tilt_serial_project
pio run --target upload --upload-port COM9
```

# Flash ESP32 on COM9

## If auto-upload fails (Waveshare script can't enter download mode)

1. **Put the ESP32 in download mode** (do this first):
   - **Hold** the **BOOT** button.
   - **Press and release** the **RESET** button.
   - **Release** the **BOOT** button.
   - The board is now in bootloader mode (you have a few seconds).

2. **Run the upload immediately** (from `pan_tilt_serial_project`):
   ```bash
   python -m esptool --port COM9 --baud 115200 --before no_reset --after hard_reset write_flash 0x1000 .pio\build\esp32dev\bootloader.bin 0x8000 .pio\build\esp32dev\partitions.bin 0x10000 .pio\build\esp32dev\firmware.bin
   ```
   Or build first if needed: `pio run` then the command above.

3. After a successful flash, the board will reset and run the new firmware. You can then run the auto-tester on COM9: from `tester/`, run `python run_tests.py COM9`.

## Normal upload (if your board enters download mode via DTR/RTS)

```bash
cd pan_tilt_serial_project
pio run --target upload --upload-port COM9
```

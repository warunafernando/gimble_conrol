# Flash Instructions for pan_tilt_base_v0.9

## Compiled Firmware Files

The firmware has been successfully compiled. The following files are ready to flash:

- **Bootloader**: `.pio\build\esp32dev\bootloader.bin` (offset: `0x1000`)
- **Partitions**: `.pio\build\esp32dev\partitions.bin` (offset: `0x8000`)
- **Application**: `.pio\build\esp32dev\firmware.bin` (offset: `0x10000`)

## Flash Command

Use the `flash_firmware.py` script from the parent directory:

```powershell
cd E:\Arduino_Controller
python .\flash_firmware.py --port COM9 --erase `
  --segment 0x1000:pan_tilt_base_v0.9\.pio\build\esp32dev\bootloader.bin `
  --segment 0x8000:pan_tilt_base_v0.9\.pio\build\esp32dev\partitions.bin `
  --segment 0x10000:pan_tilt_base_v0.9\.pio\build\esp32dev\firmware.bin
```

**Note**: Replace `COM9` with your actual COM port (check Device Manager).

## Before Flashing

1. **Put ESP32 into download mode**:
   - Hold the **BOOT** button
   - Tap the **RESET** button
   - Release **BOOT**

2. **Verify COM port**: Check Device Manager → Ports (COM & LPT) for "Silicon Labs CP210x..."

## After Flashing

1. Unplug/replug USB
2. Reboot the board
3. Test with: `python .\pc_control.py --port COM9 --center --read`

## Build Info

- **Platform**: ESP32 (Espressif 32)
- **Board**: ESP32 Dev Module
- **Framework**: Arduino
- **Flash usage**: 78.3% (1,026,625 / 1,310,720 bytes)
- **RAM usage**: 14.9% (48,876 / 327,680 bytes)

## Rebuild

To rebuild the firmware:

```powershell
cd E:\Arduino_Controller\pan_tilt_base_v0.9
python -m platformio run
```

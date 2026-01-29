# Firmware update (Waveshare 2‑Axis Pan‑Tilt + General Driver for Robots)

This kit’s “brain” is the **Waveshare General Driver for Robots** (ESP32-based). Updating “firmware” usually means **flashing new ESP32 firmware** on that driver board (not the bus servos).

Product reference: `https://www.waveshare.com/general-driver-for-robots.htm`

## 1) What you need on Windows

- **CP210x driver** (if Device Manager does not show a new COM port):
  - Waveshare CP210x driver zip: `https://files.waveshare.com/upload/d/d6/CP210x_USB_TO_UART.zip`
- **Python 3**
- This repo’s tooling:
  - `setup_windows_firmware.ps1`
  - `flash_firmware.py`

Run the setup:

```powershell
cd E:\Arduino_Controller
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
.\setup_windows_firmware.ps1
```

## 2) Put the ESP32 into download mode

1. Plug your USB cable into the board’s **Type‑C port labeled “USB”** (not the LiDAR Type‑C).
2. If the board has **BOOT/DOWNLOAD** and **RESET/EN** buttons:
   - Hold **BOOT**
   - Tap **RESET/EN**
   - Release **BOOT**

After that, Windows should show a **Silicon Labs CP210x… (COMxx)** port in Device Manager.

## 3) Get the correct firmware files

Waveshare provides firmware binaries on their wiki pages for the driver board / pan-tilt camera module.

Download the firmware package and identify whether you have:

- **A single “merged” `.bin`** (often named like `firmware.bin` / `factory.bin` / `xxx_all.bin`)
  - Flash it at **offset `0x0`**
- **Multiple `.bin` files** (bootloader/partition/app)
  - You will need the **offsets** that Waveshare provides in their instructions

If you’re unsure which you have, tell me the filenames you downloaded and I’ll map it to the correct flash command.

## 4) Flash using the script (recommended)

### Option A — single merged image (common)

```powershell
cd E:\Arduino_Controller
python .\flash_firmware.py --port COM9 --image path\to\firmware.bin --erase
```

### Option B — multiple segments (only if Waveshare gives offsets)

Example (offsets are just an example; use Waveshare’s values):

```powershell
cd E:\Arduino_Controller
python .\flash_firmware.py --port COM9 --erase `
  --segment 0x1000:path\to\bootloader.bin `
  --segment 0x8000:path\to\partitions.bin `
  --segment 0x10000:path\to\app.bin
```

## 5) Verify after flashing

- Unplug/replug USB, reboot the board.
- Confirm the COM port still appears.
- Try a simple control command from this repo:

```powershell
cd E:\Arduino_Controller
python .\pc_control.py --port COM9 --center --read
```


# Tools + quick setup (PC) for Waveshare 2‑Axis Pan‑Tilt Camera Module

This hardware uses **Waveshare “General Driver for Robots” (ESP32 + CP2102 USB‑UART)** and **ST3215 serial bus servos** (not standard PWM hobby servos). You can control it from your computer in two common ways:

- **WiFi Web UI (recommended for first-time calibration)**: connect to the device hotspot and open `192.168.4.1`.
- **USB Serial (good for scripting)**: send the same JSON commands over the ESP32 USB COM port.

## What to install on this computer (Windows)

- **Google Chrome** (for the built-in Web UI).
- **CP2102 driver** (only if the COM port doesn’t appear):
  - Download from Waveshare: `https://files.waveshare.com/upload/d/d6/CP210x_USB_TO_UART.zip`
- **Python 3** (for USB control script in this folder).
  - Install from Microsoft Store or `python.org`, then enable “Add python to PATH”.
- **Arduino IDE (optional, only if you want to reprogram the ESP32)**:
  - Arduino IDE download: `https://www.arduino.cc/en/software`
  - Board package: install **ESP32** boards (Espressif) in Arduino IDE (Boards Manager).

## Hardware connection checklist

- **Use the correct USB Type‑C port** on the driver board:
  - The “USB” Type‑C is the ESP32 programming/serial port (CP2102).
- **Power**:
  - Use the provided **12V 5A** supply into the XH2.54 power port.
  - Start with the power switch **OFF**, connect USB, then turn power **ON**.

## First-time servo calibration (Waveshare official method)

Follow the Waveshare guide here: `https://www.waveshare.com/wiki/2-Axis_Pan-Tilt_Camera_Module_Assembly_and_Configuration_Guide`

Key points:

- On the OLED, you should see **“PT CAM Version:0.9”** on boot.
- Connect your PC/phone to WiFi **`PT`** password **`12345678`**.
- Open **`http://192.168.4.1`** in Chrome.
- Set IDs (pan should be ID 2, tilt remains ID 1):
  - `{"T":501,"raw":1,"new":2}`
- Disable torque before centering by hand:
  - `{"T":210,"cmd":0}`
- Set middle position (run once for each servo id):
  - `{"T":502,"id":1}` then `{"T":502,"id":2}`
- Center command:
  - `{"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}`

## USB control from this PC (Python)

This repo includes a small script to send Waveshare JSON commands over the USB COM port.

### Install Python deps

```bash
python -m pip install -r requirements.txt
```

### Find the COM port

Open **Device Manager → Ports (COM & LPT)**. You should see something like **“Silicon Labs CP210x USB to UART Bridge (COMxx)”**.

### Send commands

Examples:

```bash
# Center the gimbal
python pc_control.py --port COM7 --center

# Set absolute angles (X: -180..180, Y: -30..90)
python pc_control.py --port COM7 --set --x 45 --y 10

# Raw JSON (advanced)
python pc_control.py --port COM7 --json "{\"T\":133,\"X\":0,\"Y\":0,\"SPD\":0,\"ACC\":0}"
```

## References (official)

- `https://www.waveshare.com/wiki/2-Axis_Pan-Tilt_Camera_Module`
- `https://www.waveshare.com/wiki/General_Driver_for_Robots`
- Product page: `https://www.waveshare.com/general-driver-for-robots.htm`

## Firmware update (ESP32 on the driver board)

If you want to **update/reflash the ESP32 firmware** on the Waveshare **General Driver for Robots**, see:

- `FIRMWARE_UPDATE.md`

This repo includes:

- `setup_windows_firmware.ps1` (installs Python flashing tools)
- `flash_firmware.py` (wrapper around `esptool`)

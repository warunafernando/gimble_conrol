# Gimbal Control

ESP32 pan-tilt gimbal controller: firmware, serial protocol, UART OTA, and web GUI with auto-connect.

---

## Firmware: Original vs New

### Original firmware

The original firmware provided the base pan-tilt and serial interface.

![Original firmware](https://raw.githubusercontent.com/warunafernando/gimble_conrol/main/pan_tilt_serial_project/docs/Original.png)

*Original FW — baseline functionality.*

---

### New firmware (this project)

The new firmware adds extended features and protocol support used by the web GUI and OTA flow.

![New firmware](https://raw.githubusercontent.com/warunafernando/gimble_conrol/main/pan_tilt_serial_project/docs/New.png)

*New FW — serial protocol (FW info with model ID & serial), UART OTA A/B, and compatibility with the auto-connect web GUI.*

---

## Project

Main code and docs are in **[pan_tilt_serial_project/](pan_tilt_serial_project/)**:

- **Firmware:** `pan_tilt_serial_project.ino` (PlatformIO, ESP32)
- **Web GUI:** `backend/` (Flask + SocketIO), auto-connects to gimbal (model ID 99)
- **Tools:** `tools/start_gui.bat`, `tools/ota_load.bat`, UART OTA scripts
- **Docs:** [Serial protocol](pan_tilt_serial_project/docs/SERIAL_PROTOCOL.md), [OTA & connection guide](pan_tilt_serial_project/docs/OTA_AND_CONNECTION_GUIDE.md)

### Quick start

```bash
cd pan_tilt_serial_project
pio run -e esp32dev                    # build
tools\ota_load.bat                    # load firmware (OTA)
tools\start_gui.bat                   # run web GUI (auto-connects to gimbal)
```

See **[pan_tilt_serial_project/README.md](pan_tilt_serial_project/README.md)** for full details.

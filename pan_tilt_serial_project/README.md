# ESP32 Pan-Tilt Gimbal Controller

Firmware and PC tools for the Waveshare ESP32-based pan-tilt gimbal: serial protocol, UART OTA updates, and a web GUI with auto-connect.

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

### Serial protocol

Binary frame format and command/response types. Full spec: [SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md).

![Serial protocol](https://raw.githubusercontent.com/warunafernando/gimble_conrol/main/pan_tilt_serial_project/docs/SerialProtocol.png)

---

## Quick start

1. **Build firmware** (PlatformIO):
   ```bash
   pio run -e esp32dev
   ```

2. **Load firmware** (UART OTA or first-time flash):
   - Double-click **`tools\ota_load.bat`** for OTA, or use manual bootloader (see [OTA and connection guide](docs/OTA_AND_CONNECTION_GUIDE.md)).

3. **Run the web GUI**:
   - Double-click **`tools\start_gui.bat`** — starts the backend and opens http://localhost:5000.  
   - The backend **auto-connects** to the gimbal (scans COM ports, connects to the device with model ID 99). No manual port selection.

---

## Repository layout

| Path | Description |
|------|-------------|
| **pan_tilt_serial_project.ino** | Main ESP32 firmware (Arduino framework). |
| **backend/** | Flask + SocketIO server: serial bridge, auto-connect, keepalive, web GUI. |
| **backend/static/index.html** | Web GUI (connection, FW info, pan/tilt, IMU, OTA, debug). |
| **tools/** | `start_gui.bat`, `uart_ota.py`, `ota_load.bat`, `get_fw_info.py`, upload scripts. |
| **docs/** | [SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md), [OTA_AND_CONNECTION_GUIDE.md](docs/OTA_AND_CONNECTION_GUIDE.md), [GIMBAL_SERIAL_CONNECTION_REFERENCE.md](docs/GIMBAL_SERIAL_CONNECTION_REFERENCE.md), UART OTA guides, IMU notes. |
| **GIMBAL_PROTOCOL.md** | Protocol summary (commands and responses). |

---

## Documentation

- **[Serial protocol (full)](docs/SERIAL_PROTOCOL.md)** — Frame format, CRC8, all command/response types, FW_INFO, OTA.
- **[OTA build and gimbal connection](docs/OTA_AND_CONNECTION_GUIDE.md)** — How to build firmware, load via UART OTA, and how the PC discovers and connects to the gimbal (auto-connect, keepalive, Rescan).
- **[Gimbal serial connection reference](docs/GIMBAL_SERIAL_CONNECTION_REFERENCE.md)** — Implementation guide to bring up gimbal serial connection in another project (probe, model_id, keep port open, parallel probe, keepalive).

---

## Requirements

- **Firmware:** PlatformIO, ESP32 Arduino framework, board support for ESP32.
- **Backend/GUI:** Python 3.7+, `pip install -r backend/requirements.txt`.
- **Hardware:** Waveshare “General Driver for Robots” (or compatible) gimbal board; USB for serial.

---

## Images (Original.png / New.png)

Place your screenshots or diagrams in the **`docs/`** folder:

- **`docs/Original.png`** — Original firmware (e.g. version or UI).
- **`docs/New.png`** — New firmware (e.g. added features or version with model ID/serial).

The README above references them; add the files and they will show on GitHub.

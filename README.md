# Pan-Tilt Controller (ESP32 + ST3215 Servos)

Custom firmware and tools for the **Waveshare 2-Axis Pan-Tilt Camera Module** using **ESP32** and **ST3215 serial bus servos**.

## Project Structure

```
Arduino_Controller/
├── pan_tilt_serial_project/     # Main firmware and tools
│   ├── *.ino, *.cpp, *.h        # ESP32 firmware (PlatformIO)
│   ├── backend/                  # Python Web GUI backend
│   │   ├── app.py               # Flask-SocketIO server
│   │   ├── protocol.py          # Binary protocol encoder/decoder
│   │   └── static/index.html    # Web GUI frontend
│   ├── tools/                    # Utilities
│   │   ├── uart_ota.py          # OTA firmware upload tool
│   │   ├── pan_tilt_test_gui.py # Desktop test GUI
│   │   └── waveshare_uploader.py# Initial flash tool
│   └── docs/                     # Documentation
│       ├── NEW_PANTILT_DEVICE_GUIDE.md  # Factory setup guide
│       ├── UART_OTA_BUILD_GUIDE.md      # OTA build guide
│       └── UART_OTA_LOAD_GUIDE.md       # OTA load guide
├── SERIAL_PAN_TILT_SPEC.md      # Protocol specification
├── ST3215_MEMORY_REGISTER_MAP.md # Servo register reference
└── ST3215 memory register map-EN.xls
```

## Features

- **Custom Binary Protocol** over UART (921600 baud) with CRC8 checksums
- **OTA Firmware Updates** via serial (no bootloader button required after initial flash)
- **A/B Image Scheme** for safe firmware updates with rollback capability
- **Web GUI** for real-time control and monitoring
- **IMU Integration** (QMI8658C accelerometer/gyroscope + AK09918C magnetometer)
- **Power Monitoring** (INA219)
- **ST3215 Servo Control** with position/torque feedback

## Quick Start

### 1. Initial Setup (New Device)

See `pan_tilt_serial_project/docs/NEW_PANTILT_DEVICE_GUIDE.md` for complete factory-to-production setup.

### 2. Flash Firmware

**First time** (requires manual boot mode):
```bash
cd pan_tilt_serial_project/tools
pip install -r requirements.txt
python waveshare_uploader.py --port COM9
```

**Subsequent updates** (OTA - recommended):
```bash
python uart_ota.py --port COM9 --firmware ../.pio/build/esp32s3_ota/firmware.bin
```

### 3. Run Web GUI

```bash
cd pan_tilt_serial_project/backend
pip install -r requirements.txt
python app.py
```

Open `http://localhost:5000` in browser.

## Hardware

- **Board**: Waveshare General Driver for Robots (ESP32-S3)
- **Servos**: ST3215 serial bus servos (1 Mbps half-duplex TTL)
- **IMU**: QMI8658C + AK09918C (I2C)
- **Power**: INA219 monitor (I2C)
- **USB**: CP2102 USB-UART bridge

### Connections

| Component | Interface | Notes |
|-----------|-----------|-------|
| Servos    | UART1 (GPIO17/18) | 1 Mbps half-duplex |
| IMU       | I2C (GPIO11/12) | QMI8658C + AK09918C |
| INA219    | I2C (GPIO11/12) | Power monitor |
| Host PC   | UART0 (USB) | 921600 baud |

## Protocol Overview

Binary frame format:
```
[STX] [LEN_L] [LEN_H] [SEQ_L] [SEQ_H] [TYPE_L] [TYPE_H] [PAYLOAD...] [CRC8] [ETX]
```

Key commands:
- `100` - Move (pan/tilt angles)
- `200` - Get IMU data
- `300` - Get power data
- `500/501` - OTA start/chunk
- `610/611` - Get FW info / Switch FW slot

See `SERIAL_PAN_TILT_SPEC.md` for full protocol specification.

## References

- [Waveshare 2-Axis Pan-Tilt Camera Module](https://www.waveshare.com/wiki/2-Axis_Pan-Tilt_Camera_Module)
- [Waveshare General Driver for Robots](https://www.waveshare.com/wiki/General_Driver_for_Robots)
- [ST3215 Servo Datasheet](https://www.waveshare.com/wiki/ST3215_Servo)

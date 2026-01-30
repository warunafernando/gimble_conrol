# Gimbal PC Backend

Runs on the PC: connects to the ESP32 over serial (binary protocol per GIMBAL_PROTOCOL.md) and pushes data to a web GUI via WebSocket.

## Features

- **Serial bridge**: Binary frame RX/TX to ESP32 at 921600 baud
- **WebSocket push**: Live pan/tilt/torque (from ACK_EXECUTED), IMU (type 1002), connection state
- **Debug**: Ring buffer of last 500 log lines (serial open/close, RX/TX, parsed frames); debug panel in web GUI with refresh and live tail

## Setup

```bash
cd backend
pip install -r requirements.txt
```

## Run

```bash
python app.py [PORT]
```

Example: `python app.py COM3` (Windows) or `python app.py /dev/ttyUSB0` (Linux).

Then open http://localhost:5000 in a browser. Use the Connection panel to set port/baud and Connect. Use Commands to send Enter tracking (137) or Get IMU (126). Debug log shows serial activity.

## Protocol

See `../GIMBAL_PROTOCOL.md`. Backend uses same binary framing (STX/LEN/SEQ/TYPE/PAYLOAD/CRC8/ETX) and CRC8.

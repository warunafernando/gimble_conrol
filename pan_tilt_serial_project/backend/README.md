# Gimbal PC Backend

Runs on the PC: connects to the ESP32 over serial (binary protocol per GIMBAL_PROTOCOL.md) and serves a web GUI with WebSocket push.

## Features

- **Auto-connect**: On startup, scans COM ports and connects to the first gimbal with model ID 99 (keeps port open; no close/reopen).
- **Serial bridge**: Binary frame RX/TX to ESP32 at 921600 baud.
- **WebSocket push**: Live pan/tilt, IMU, connection state, FW info.
- **Keepalive**: Every 30 s checks connection; disconnects if no response.
- **Debug**: Ring buffer of last 500 log lines; debug panel in web GUI.

## Setup

```bash
cd backend
pip install -r requirements.txt
```

## Run

```bash
python app.py
```

No port argument: the backend discovers the gimbal automatically. Then open http://localhost:5000 in a browser. The Connection & FW Info table shows COM port, status (Connected / Not connected), FW version, serial number, and a Disconnect button. Use **Rescan and connect** if you unplug and replug the gimbal.

## Protocol

See `../GIMBAL_PROTOCOL.md`. Backend uses same binary framing (STX/LEN/SEQ/TYPE/PAYLOAD/CRC8/ETX) and CRC8.

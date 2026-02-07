# OTA Build and Gimbal Connection Guide

This guide covers:

1. **How to build firmware** for UART OTA (and manual flash).
2. **How to load firmware** via UART OTA (or manual bootloader).
3. **How the PC connects to the gimbal** — auto-discovery, probing, keepalive, and using the web GUI.

---

## Part 1: Building Firmware (OTA-Ready)

### 1.1 Prerequisites

- **Hardware:** Waveshare ESP32 “General Driver for Robots” (or compatible) gimbal board.
- **Software:** Python 3, [PlatformIO](https://platformio.org/) (CLI or IDE), and project dependencies.

### 1.2 Partition Scheme (A/B OTA)

The firmware uses two app partitions:

- **ota_0** (slot A) and **ota_1** (slot B) — alternate firmware slots.
- **otadata** — stores which slot to boot next.

Check `partitions_4mb.csv` (or `partitions.csv` for 8 MB) in the project root. The build uses these so that UART OTA writes to the **inactive** slot; after verification, the device commits and reboots into the new image.

### 1.3 Build Commands

From the project root:

```bash
cd e:\Arduino_Controller\pan_tilt_serial_project
pio run -e esp32dev
```

Output:

- `.pio/build/esp32dev/firmware.bin` — main application (used for OTA).
- `.pio/build/esp32dev/bootloader.bin`
- `.pio/build/esp32dev/partitions.bin`

The version string is taken from `version.txt` (or build flags) and embedded in the firmware; it appears in **GET_FW_INFO** (2610) as `version_a` / `version_b`.

### 1.4 Clean Build

If you change partitions or suspect stale build artifacts:

```bash
pio run -e esp32dev -t clean
pio run -e esp32dev
```

---

## Part 2: Loading Firmware (UART OTA)

UART OTA updates the ESP32 over the same USB serial link at **921600 baud**, without holding BOOT or RESET.

### 2.1 Requirements

- ESP32 must already be running **OTA-capable** firmware at least once (first time use manual bootloader).
- Correct COM port (e.g. COM9); no other program should have the port open.

### 2.2 Recommended: ota_load.bat

Double-click **`tools\ota_load.bat`** (or run from a shell). It:

- Builds firmware if needed.
- Runs the UART OTA tool in a separate window (avoids IDE/shell timeout).
- Uses default port (e.g. COM9) unless you pass a port: `tools\ota_load.bat COM9`.

### 2.3 Command-Line OTA

```bash
# Default port, auto-detect firmware
python tools/uart_ota.py -p COM9

# Explicit firmware file
python tools/uart_ota.py -p COM9 -f .pio/build/esp32dev/firmware.bin

# Verbose
python tools/uart_ota.py -p COM9 -v

# Different baud (default 921600)
python tools/uart_ota.py -p COM9 -b 115200
```

### 2.4 OTA + Manual Fallback: load_new_fw.bat

If OTA might not be available (e.g. first flash or broken OTA), use:

```bash
tools\load_new_fw.bat COM9
```

This tries UART OTA first; if it fails, you can then put the ESP32 in bootloader mode and the script (or you) can use the manual flash method below.

### 2.5 Manual Bootloader (First Flash or OTA Failed)

1. **Enter bootloader:** Hold **BOOT**, press and release **RESET**, release **BOOT**.
2. **Flash immediately:**

   ```bash
   python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev --manual
   ```

   Or with esptool:

   ```bash
   python -m esptool --chip esp32 --port COM9 --baud 460800 --before no_reset --after hard_reset write_flash 0x1000 .pio/build/esp32dev/bootloader.bin 0x8000 .pio/build/esp32dev/partitions.bin 0x10000 .pio/build/esp32dev/firmware.bin
   ```

### 2.6 OTA Flow Summary

1. Tool opens COM port at 921600.
2. Sends **OTA_START** (size, hash type, CRC32/SHA256).
3. ESP32 replies **OTA_STARTED** (inactive slot, size).
4. Tool sends **OTA_CHUNK** (offset, length, data) until full image sent; ESP32 replies progress.
5. Tool sends **OTA_END**. ESP32 verifies checksum, commits, reboots.
6. Transfer typically **~40–60 seconds** for ~300 KB at 921600.

See **`docs/SERIAL_PROTOCOL.md`** and **`GIMBAL_PROTOCOL.md`** for OTA command/response details.

---

## Part 3: How the PC Connects to the Gimbal

The **web GUI** is served by a **Flask backend** that discovers and connects to the gimbal automatically. You do **not** select a COM port manually in the GUI.

### 3.1 Start the Backend and GUI

**Option A — One-click (recommended)**

Double-click **`tools\start_gui.bat`**. It:

- Starts the backend from `backend/` (`python app.py`).
- Waits a few seconds.
- Opens http://localhost:5000 in your browser.

**Option B — Manual**

```bash
cd pan_tilt_serial_project\backend
pip install -r requirements.txt
python app.py
```

Then open http://localhost:5000 in a browser.

### 3.2 Auto-Connect (Discovery)

When the backend starts:

1. **Scan:** It enumerates all COM ports (e.g. COM3, COM4, COM5, COM9, COM13).
2. **Probe (parallel):** For each port, it opens the port, sends **GET_FW_INFO** (610), and waits for a **FW_INFO** (2610) response (timeout ~2 s). Ports are probed **in parallel** so a slow or unresponsive port (e.g. COM3) does not delay discovery.
3. **Identify gimbal:** The firmware returns **FW_INFO** with a **model_id** byte (e.g. **99** for this gimbal). The backend accepts only devices with **model_id == 99**.
4. **Keep port open:** When a device responds with model_id 99, the backend **does not close** that port. It passes the already-open serial handle to the serial bridge and uses it for all further communication.
5. **First match wins:** The first port that reports model 99 is used; the backend then ignores other probe results and closes any other ports it opened during probing.

So if the gimbal is on **COM9**, the backend will connect to COM9 as soon as COM9 responds (often within ~0.5 s), even if COM3 or others are slow or unavailable.

### 3.3 Connection State in the GUI

The **Connection & FW Info** panel shows:

- **COM port** — The port in use (e.g. COM9) or "—" if not connected.
- **Status** — "Connected" (green) or "Not connected".
- **FW version** — Active firmware version from the gimbal (from FW_INFO).
- **Serial** — Unique serial number from the gimbal (from FW_INFO).
- **Disconnect** — Button to close the serial connection. After disconnect, the backend will try to discover a gimbal again on the next scan cycle (~5 s) or when you click **Rescan and connect**.

### 3.4 Rescan and Connect

If you unplug the gimbal and plug it back in (possibly on a different COM port), or if the first scan did not find it:

- Click **Rescan and connect** in the GUI.  
- This triggers the backend to run the discovery process again (list COM ports, probe each for model 99, connect to the first match).

No manual port selection is required.

### 3.5 Keepalive (Stay-Connected Check)

While connected, the backend runs a **keepalive** every **30 seconds**:

1. It sends **GET_FW_INFO** (610) to the gimbal.
2. When any **FW_INFO** (2610) is received, the backend clears an internal “ping pending” flag.
3. If no FW_INFO is received within **5 seconds** of that ping, the backend assumes the link is dead (e.g. cable unplugged, gimbal reset) and **disconnects**. The GUI then shows "Not connected"; you can use **Rescan and connect** after reconnecting the device.

### 3.6 Protocol and Port Details

- **Baud rate:** 921600.
- **Protocol:** Binary frames per **`docs/SERIAL_PROTOCOL.md`** and **`GIMBAL_PROTOCOL.md`** (STX/LEN/SEQ/TYPE/PAYLOAD/CRC8/ETX).
- **Model ID 99:** Used by this product so the host can distinguish the gimbal from other serial devices (e.g. other boards on COM3, COM4).

---

## Part 4: Troubleshooting

| Issue | What to try |
|-------|-------------|
| GUI shows "Not connected" | Ensure gimbal is plugged in and firmware reports model_id 99. Click **Rescan and connect**. Check Debug panel for "Auto-connect: scanning ..." and probe messages. |
| Backend won’t start | Run from `backend/`: `pip install -r requirements.txt` then `python app.py`. Check port 5000 is free. |
| OTA "No response to OTA_START" | Wrong COM port, or ESP32 not running OTA firmware. First flash via manual bootloader. |
| OTA "CHECKSUM_FAIL" | Rebuild firmware (`pio run -e esp32dev`), don’t modify the .bin; ensure same hash type (CRC32/SHA256) as firmware. |
| OTA times out in IDE | Use **`tools\ota_load.bat`** (double-click) so OTA runs in a separate window. |
| Port in use | Close Arduino Serial Monitor, other terminals, or scripts using the same COM port. Disconnect in the GUI before running OTA or other tools. |

---

## Part 5: File Reference

| File | Purpose |
|------|---------|
| `tools/start_gui.bat` | Start backend and open web GUI. |
| `tools/ota_load.bat` | Build + UART OTA in separate window (no timeout). |
| `tools/load_new_fw.bat` | OTA first, then optional manual bootloader. |
| `tools/uart_ota.py` | UART OTA upload script. |
| `tools/waveshare_uploader.py` | Manual bootloader upload. |
| `backend/app.py` | Flask + SocketIO server; auto-connect, keepalive, serial bridge. |
| `backend/protocol.py` | Frame build/parse, CRC8, command/response constants. |
| `docs/SERIAL_PROTOCOL.md` | Full serial protocol specification. |
| `docs/GIMBAL_SERIAL_CONNECTION_REFERENCE.md` | Reference: how to connect to gimbal serial (probe, model_id, keep port open; bring-up in another project). |
| `docs/UART_OTA_BUILD_GUIDE.md` | Detailed OTA build and load steps. |
| `docs/UART_OTA_LOAD_GUIDE.md` | Short “how to load firmware” steps. |
| `GIMBAL_PROTOCOL.md` | Protocol summary (commands/responses). |

---

*Document version: 1.0 | OTA build and gimbal connection*

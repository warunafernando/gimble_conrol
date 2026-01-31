# New Pan-Tilt Device: Factory to Production Guide

High-level guide for transforming a new pan-tilt device from factory state to the production binary-protocol gimbal with OTA support.

---

## 1. Overview

| Aspect | Original (Factory) | Target (Production) |
|--------|--------------------|---------------------|
| **Hardware** | Waveshare "General Driver for Robots" (ESP32 + ST3215 servos + IMU + INA219) | Same hardware |
| **Firmware** | `pan_tilt_base_v0.9` (Waveshare demo) | `pan_tilt_serial_project` |
| **Communication** | JSON over UART @ 115200, WiFi, HTTP, ESP-NOW | Binary protocol over UART @ 921600 only |
| **Partitions** | Default (single app) | A/B OTA (ota_0, ota_1) |
| **Updates** | Manual reflash only | UART OTA (no bootloader) |
| **Host** | Web UI, JSON scripts | PC backend + Web GUI, Python tools |

---

## 2. Original State (From Factory)

### 2.1 Hardware

- **Board:** Waveshare ESP32 "General Driver for Robots"
- **USB:** Middle-side USB port (CP2102, e.g. COM9) — *not* the LIDAR Type-C port
- **Servos:** 2× ST3215 (1 Mbps, RX=18, TX=19)
- **IMU:** Depends on board; original firmware may use Adafruit ICM20948
- **Power monitor:** INA219 (I2C)
- **Buttons:** BOOT (download), RESET

### 2.2 Original Firmware (`pan_tilt_base_v0.9`)

| Feature | Description |
|---------|-------------|
| **Protocol** | JSON over UART @ 115200 |
| **Modules** | WiFi, WebServer, LittleFS, OLED, ESP-NOW, RoArm-M2 |
| **Partitions** | Default (no OTA) |
| **Update** | Manual flash only (bootloader mode) |
| **Config** | wifiConfig.json, devConfig.json in LittleFS |

### 2.3 What You Get Out of the Box

- Device boots into Waveshare demo firmware
- May have WiFi, HTTP server, JSON UART commands
- No binary gimbal protocol, no OTA
- Servo IDs may be 1 and 2 (verify with PING)

---

## 3. Target State (Production)

### 3.1 Firmware (`pan_tilt_serial_project`)

| Feature | Description |
|---------|-------------|
| **Protocol** | Binary frames: `STX LEN SEQ TYPE PAYLOAD CRC8 ETX` @ 921600 baud |
| **Partitions** | A/B OTA (ota_0, ota_1, otadata) |
| **Update** | UART OTA (no BOOT+RESET after first flash) |
| **States** | IDLE, TRACKING, CONFIG |
| **Sensors** | QMI8658C + AK09918C (IMU), INA219 (power) |
| **Servos** | ST3215 ID 1 = pan, ID 2 = tilt |

### 3.2 Serial Protocol (Summary)

- **Frame format:** [STX=0x02][LEN][SEQ][TYPE][PAYLOAD][CRC8][ETX=0x03]
- **Checksum:** CRC8 over LEN..PAYLOAD (poly 0x07, init 0x00)
- **Max payload:** 251 bytes
- **Commands:** Move (133–135, 172–175), state (137, 139, 140), IMU (126), INA (160), OTA (600–603), FW info (610–611), config (200, 210–213, 501, 502), etc.
- **Responses:** ACK_RECEIVED (1), ACK_EXECUTED (2), NACK (3), IMU (1002), INA (1010), OTA (2600–2603), FW_INFO (2610), etc.

Full spec: [GIMBAL_PROTOCOL.md](../GIMBAL_PROTOCOL.md)

### 3.3 OTA (Over-The-Air Update)

- **A/B scheme:** OTA writes to the inactive slot
- **Flow:** OTA_START → OTA_CHUNK (×N) → OTA_END → verify CRC32 → commit → reboot
- **Tool:** `python tools/uart_ota.py -p COM9` or `tools\ota_load.bat`
- **Fallback:** Manual bootloader (hold BOOT, press RESET) if OTA fails

Full spec: [UART_OTA_BUILD_GUIDE.md](UART_OTA_BUILD_GUIDE.md), [UART_OTA_LOAD_GUIDE.md](UART_OTA_LOAD_GUIDE.md)

---

## 4. Transformation Steps (High Level)

### Step 1: Prepare Host Environment

1. **Install tools**
   - Python 3, pyserial
   - PlatformIO (VS Code extension or CLI)
   - CP2102 driver (if not present)

2. **Get project**
   - Clone or copy `pan_tilt_serial_project`
   - Install dependencies: `pip install -r backend/requirements.txt` (for Web GUI)
   - Install PlatformIO libs: `pio run` (auto-downloads)

### Step 2: Connect Hardware

1. Connect USB to the **middle-side USB port** (not LIDAR)
2. Check Device Manager for COM port (e.g. COM9)
3. Power servos externally (6–12.6 V) if required

### Step 3: First-Time Flash (Manual Bootloader)

**Required once** — the device has no OTA-capable firmware yet.

1. **Build firmware**
   ```bash
   cd pan_tilt_serial_project
   pio run -e esp32dev
   ```

2. **Put ESP32 in bootloader mode**
   - Hold **BOOT**
   - Press and release **RESET**
   - Release **BOOT**

3. **Flash immediately**
   ```bash
   python tools/waveshare_uploader.py -p COM9 --build-dir .pio/build/esp32dev --manual
   ```
   Or:
   ```bash
   python -m esptool --chip esp32 --port COM9 --baud 460800 --before no_reset --after hard_reset write_flash 0x1000 .pio/build/esp32dev/bootloader.bin 0x8000 .pio/build/esp32dev/partitions.bin 0x10000 .pio/build/esp32dev/firmware.bin
   ```

4. ESP32 reboots into the new firmware (slot A or B).

### Step 4: Verify Binary Protocol

1. **Start Web GUI**
   ```bash
   cd backend
   python app.py
   ```
   Open http://localhost:5000

2. **Connect** to COM9 @ 921600
3. **Get FW Info** — should show version and active slot
4. **Get state (144)** — should return IDLE
5. **Enter tracking (137)** — optional, for move tests

### Step 5: Servo Setup (If Needed)

If servos have wrong IDs or need calibration:

1. **Enter config** (139)
2. **PING (200)** — verify IDs 1 (pan) and 2 (tilt)
3. **Set ID (501)** — change ID if required (from_id, to_id)
4. **Calibrate (502)** — set current position as center (per servo)
5. **Exit config** (140)

### Step 6: Subsequent Updates (UART OTA)

After first flash, **always use UART OTA** for firmware updates:

1. **Build**
   ```bash
   pio run -e esp32dev
   ```

2. **Run OTA** (close Web GUI or disconnect serial first)
   ```bash
   tools\ota_load.bat COM9
   ```
   Or:
   ```bash
   python tools/uart_ota.py -p COM9
   ```

3. Wait ~45–60 s; ESP32 reboots into new firmware.

### Step 7: Optional — Mirror Firmware to Both Slots

To have the same firmware in both A and B:

1. Run OTA once (writes to inactive slot)
2. Optionally run OTA again to write the same image to the other slot
3. Use **SWITCH_FW (611)** from host to switch boot slot if needed

---

## 5. File Reference

| File / Dir | Purpose |
|------------|---------|
| `pan_tilt_serial_project.ino` | Main firmware |
| `GIMBAL_PROTOCOL.md` | Binary protocol spec |
| `docs/UART_OTA_BUILD_GUIDE.md` | Build and OTA details |
| `docs/UART_OTA_LOAD_GUIDE.md` | OTA load steps |
| `tools/uart_ota.py` | OTA upload tool |
| `tools/ota_load.bat` | OTA script (no timeout) |
| `tools/load_new_fw.bat` | OTA + manual fallback |
| `tools/waveshare_uploader.py` | Manual bootloader upload |
| `backend/app.py` | Web GUI backend |
| `backend/static/index.html` | Web GUI frontend |
| `partitions_4mb.csv` | OTA A/B partition table |

---

## 6. Quick Checklist

- [ ] Host: Python, pyserial, PlatformIO, CP2102 driver
- [ ] USB connected to correct port (COM9 or similar)
- [ ] First flash: manual bootloader (BOOT + RESET)
- [ ] Verify: Web GUI connect, Get FW Info, Get state
- [ ] Servo IDs: 1=pan, 2=tilt (PING, Set ID if needed)
- [ ] Calibrate servos if required (502)
- [ ] Future updates: UART OTA only (no bootloader)

---

## 7. Troubleshooting

| Problem | Solution |
|---------|----------|
| COM port not found | Check USB cable, CP2102 driver, Device Manager |
| Port in use | Disconnect Web GUI, serial monitors; wait and retry |
| OTA fails / no response | First-time flash requires manual bootloader |
| CHECKSUM_FAIL on OTA | Clean build (`pio run -t clean`), rebuild |
| Servos don't move | Verify IDs, calibration, power supply |
| Wrong IMU data | Confirm board has QMI8658C + AK09918C |

---

*Last updated: Jan 2025*

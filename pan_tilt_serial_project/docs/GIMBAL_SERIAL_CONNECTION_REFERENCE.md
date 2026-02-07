# How to Connect to the Gimbal Serial Port — Reference Guide

This document describes **how the backend discovers and connects to the gimbal over serial**. Use it as a reference to bring up the same behavior in another project (different language, GUI, or platform).

---

## 1. Goal

- **No manual port selection:** The PC should find the gimbal automatically.
- **Correct device only:** Connect only to our gimbal (identified by **model_id** in the firmware response), not to other serial devices (e.g. Arduino, GPS).
- **Single connection:** One open serial port; keep it open for the session.
- **Reconnect:** Support “rescan” when the user unplugs/replugs the gimbal (possibly on a different COM port).
- **Stay-connected check:** Detect when the link is lost (e.g. unplug) and disconnect so the UI can show “Not connected” and offer rescan.

---

## 2. Prerequisites

- **Protocol:** Binary frames (STX, LEN, SEQ, TYPE, PAYLOAD, CRC8, ETX). See [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) and [GIMBAL_PROTOCOL.md](../GIMBAL_PROTOCOL.md).
- **Baud rate:** 921600.
- **Firmware:** Device must respond to **GET_FW_INFO** (command 610) with **FW_INFO** (response 2610). The payload must include **model_id** (e.g. 99) so the host can tell “this is our gimbal.”
- **FW_INFO payload (70-byte format):**  
  `active_slot(1)` `serial(4 LE)` `model_id(1)` `version_a(32)` `version_b(32)`.

---

## 3. High-Level Flow

```
1. Enumerate COM ports (e.g. COM3, COM4, COM5, COM9, COM13).
2. For each port (optionally in parallel):
   a. Open the port at 921600.
   b. Send one GET_FW_INFO frame.
   c. Read until a valid FW_INFO frame or timeout (~2 s).
   d. Parse payload; if model_id == TARGET (e.g. 99):
      - Do NOT close the port.
      - Return (port_name, fw_info_dict, open_serial_handle).
   e. Else: close the port and continue.
3. Take the first port that returned model_id == TARGET.
4. Pass the open serial handle to your “serial bridge” or main I/O loop (do not close and reopen).
5. Optionally: run a keepalive (e.g. every 30 s send GET_FW_INFO; if no response in 5 s, disconnect).
6. On disconnect or “rescan,” close the handle and go back to step 1.
```

---

## 4. Implementation Details

### 4.1 Windows Port Name

On Windows, open ports as **`\\.\COMn`** (e.g. `\\.\COM9`) so COM10+ and some drivers work reliably.

```python
# Python example
import platform
def serial_port_name(port: str) -> str:
    port = (port or "").strip()
    if not port:
        return port
    if platform.system() == "Windows":
        pu = port.upper()
        if pu.startswith("COM") and not pu.startswith("\\\\"):
            return "\\\\.\\" + pu
    return port
```

### 4.2 Probe One Port

- Open port with a **short read timeout** (e.g. 0.05 s).
- Clear the input buffer, send **one** GET_FW_INFO frame (use your protocol’s `build_frame(seq, CMD_GET_FW_INFO, b"")`).
- Loop until timeout (e.g. 2 s): read bytes, look for **STX** (0x02), parse frame (LEN, SEQ, TYPE, PAYLOAD, CRC8, ETX). Ignore **ACK_RECEIVED** (type 1); when you get **FW_INFO** (type 2610), decode the payload.
- **Decode FW_INFO:**  
  If payload length ≥ 70: `model_id = payload[5]`, `serial = uint32_le(payload[1:5])`, `version_a = payload[6:38].rstrip(b'\x00').decode()`, `version_b = payload[38:70].rstrip(b'\x00').decode()`.
- If `model_id == TARGET` (e.g. 99): return the FW info and **the open serial object** (do not close it).  
  Otherwise: close the port and return “not found.”

Reference in this project: **`backend/app.py`** — `probe_port_for_gimbal()`, **`backend/protocol.py`** — `build_frame()`, `parse_frame()`, `decode_fw_info()`.

### 4.3 Keep the Port Open

Do **not** close the port after a successful probe and then open it again. On Windows the port can stay locked for a short time, so the second open may fail. Instead:

- Return the **open serial handle** from the probe.
- Your main serial reader/writer (e.g. “serial bridge”) should accept an **optional existing serial** and use it instead of opening by name.

Example (conceptual):

```python
# Serial bridge
def start(self, existing_serial=None):
    if existing_serial and existing_serial.is_open:
        self._serial = existing_serial
        self._serial.timeout = 0.01   # for read loop
        # start read thread...
        return True
    # else open by port name as before
    self._serial = serial.Serial(port_name, baud, timeout=0.01)
    # ...
```

Reference: **`backend/serial_bridge.py`** — `start(existing_serial=...)`.

### 4.4 Parallel Probing

Probe **all** COM ports in parallel (e.g. one thread per port). Use the **first** port that returns `model_id == TARGET`. Benefits:

- A slow or stuck port (e.g. “semaphore timeout”) does not block discovery.
- If the gimbal is on COM9, you get it as soon as COM9 responds (~0.5 s), instead of waiting for COM3, COM4, COM5 to time out.

After you have a winner, drain the other threads’ results and **close** any other serial handles they opened, so you don’t leave ports open.

Reference: **`backend/app.py`** — `_auto_connect_worker()`, `_probe_one()`, result queue and “drain_rest.”

### 4.5 Sort COM Ports Numerically

Sort port names by number (COM3, COM4, COM9, COM13) instead of string order (COM13 before COM3). Optional but improves user expectations and log order.

```python
import re
def com_sort_key(port):
    m = re.match(r"^COM(\d+)$", (port or "").upper())
    return (int(m.group(1)), port) if m else (99999, port)
# ports = sorted(ports, key=com_sort_key)
```

### 4.6 Keepalive (Optional but Recommended)

In a background timer (e.g. every 30 s):

1. Send **GET_FW_INFO**.
2. Set a “ping sent” timestamp.
3. When **any** FW_INFO is received in the normal read path, clear that timestamp.
4. After 5 s, if the timestamp is still set, consider the link dead: close the serial handle and set state to “disconnected.” The UI can then show “Not connected” and offer “Rescan.”

Reference: **`backend/app.py`** — `_keepalive_worker()`, and in `on_frame` for type 2610 clearing `ping_sent_at`.

### 4.7 Emitting State from Background Threads

If your UI runs in another thread (e.g. Flask/SocketIO), do **not** call “emit” or “send to UI” directly from the serial or probe thread. Use a **queue**: the serial/probe thread pushes (e.g. “state”, state_dict) to a queue, and a single “emit worker” thread pulls from the queue and sends to the UI. This avoids crashes and races.

Reference: **`backend/app.py`** — `_emit_queue`, `_emit_worker()`, and `_emit_queue.put_nowait(("state", dict(_state)))` wherever connection state changes.

---

## 5. Constants and Protocol References

| Item | Value / location |
|------|-------------------|
| GET_FW_INFO command | 610 |
| FW_INFO response | 2610 |
| Model ID (this gimbal) | 99 |
| Baud rate | 921600 |
| FW_INFO payload (with model_id) | 70 bytes: slot(1), serial(4), model_id(1), version_a(32), version_b(32) |
| Probe timeout | ~2 s |
| Keepalive interval | 30 s |
| Keepalive “no response” timeout | 5 s |

Frame format and CRC8: [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md).  
Payload layout: **`backend/protocol.py`** — `decode_fw_info()`, `FW_INFO_PAYLOAD_WITH_MODEL`.

---

## 6. File Reference (This Project)

| File | Purpose |
|------|---------|
| **backend/app.py** | Auto-connect loop, `probe_port_for_gimbal()`, parallel probe, keepalive, state queue, `run_bridge(port, existing_serial=...)`. |
| **backend/serial_bridge.py** | `start(existing_serial=...)`, read loop, send_command; owns the serial handle after connect. |
| **backend/protocol.py** | `build_frame()`, `parse_frame()`, `decode_fw_info()`, `cmd_get_fw_info()`, STX, RSP_FW_INFO, CMD_GET_FW_INFO. |
| **docs/SERIAL_PROTOCOL.md** | Full frame and FW_INFO specification. |
| **docs/OTA_AND_CONNECTION_GUIDE.md** | User-facing “how to connect” and OTA. |

---

## 7. Minimal Bring-Up Checklist

To replicate this connection approach in another project:

1. Implement or reuse **frame build/parse and CRC8** (see SERIAL_PROTOCOL.md).
2. Implement **GET_FW_INFO** (command 610) and **decode FW_INFO** (70-byte payload, model_id at byte 5).
3. **Enumerate COM ports**; optionally sort by COM number.
4. **Probe:** open port → send GET_FW_INFO → read until FW_INFO or timeout → decode; if model_id == 99, return open handle; else close.
5. **Do not close and reopen** the winning port; pass the open handle to your main serial reader.
6. Optionally: **parallel probe** and **keepalive** as above.
7. Optionally: **state queue** so connection state updates are sent to the UI from a single thread.

With these in place, the gimbal serial connection can be brought up the same way as in this backend.

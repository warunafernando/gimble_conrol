# UART OTA Plan for Waveshare ESP32 Pan-Tilt Controller

**Objective:** Enable firmware updates over UART (USB serial) **without** requiring manual BOOT+RESET. The running firmware receives new firmware chunks over the existing serial link and flashes them to an OTA partition, then reboots into the new image.

---

## 1. Overview

| Item | Description |
|------|--------------|
| **Target** | ESP32-S3-DevKitC-1 (per schematic; Waveshare pan_tilt_serial_project) |
| **MCU** | ESP32-S3-WROOM-1-N4R2 (dual-core, WiFi + BLE) |
| **Flash** | **8 MB** (W25Q64JVSSIQ, 64 Mbit SPI flash) |
| **PSRAM** | **8 MB** (ESP-PSRAM64H, octal PSRAM) |
| **Interface** | UART0 via CH343E (USB, 921600 baud) – same as gimbal protocol |
| **Scheme** | **A/B images** – two app slots; write to inactive, verify, commit, reboot |
| **Safety** | **Commit only after checksum verification** – bad image never activated |
| **Benefit** | No manual bootloader mode; rollback on failure; safe in-field updates |
| **Compatibility** | Keeps existing binary protocol; OTA is an add-on command mode |

---

## 2. Architecture

### 2.1 A/B Image Scheme (8 MB Flash)

The ESP32 flash (**8 MB total**) uses **two app slots** (A and B). At any time, one slot is **active** (running) and the other is **inactive** (target for OTA). After a successful OTA, the inactive slot becomes active on reboot.

```
┌───────────────────────────────────────────────────────────────┐
│                    ESP32 Flash (8 MB = 0x800000)              │
├───────────────────────────────────────────────────────────────┤
│  nvs        │  0x9000   │  NVS storage (24 KB)                │
│  otadata    │  0xD000   │  OTA state (8 KB)                   │
│  ota_0 (A)  │  0x10000  │  App slot A (2 MB)                  │
│  ota_1 (B)  │  0x210000 │  App slot B (2 MB)                  │
│  spiffs     │  0x410000 │  Filesystem (~4 MB)                 │
└───────────────────────────────────────────────────────────────┘
```

| Term | Meaning |
|------|---------|
| **Active slot** | The slot currently running (e.g. A) |
| **Inactive slot** | The target for the next OTA (e.g. B) |
| **otadata** | Partition that tracks which slot to boot |
| **Commit** | Mark the new image as valid and set it as boot target |

### 2.2 OTA Flow (Commit After Checksum)

```
┌─────────────────┐                                      ┌─────────────────────┐
│  Host (PC)      │         UART (binary protocol)       │  ESP32 (running FW)  │
│  - OTA tool     │ ──────────────────────────────────►  │  - Active: slot A    │
└─────────────────┘                                      └─────────────────────┘
        │                                                         │
        │  1. OTA_START (size, checksum_type, expected_hash)      │
        │ ──────────────────────────────────────────────────────► │
        │                                                         │
        │         ACK_OTA_STARTED (inactive_slot = B)             │
        │ ◄────────────────────────────────────────────────────── │
        │                                                         │
        │  2. OTA_CHUNK (offset, len, data) × N                   │
        │ ──────────────────────────────────────────────────────► │  Write to slot B
        │         ACK_OTA_CHUNK (bytes_written, progress)         │
        │ ◄────────────────────────────────────────────────────── │
        │         ...                                             │
        │                                                         │
        │  3. OTA_END (final_hash)                                │
        │ ──────────────────────────────────────────────────────► │
        │                                                         │
        │         [ESP32 computes hash of slot B]                 │
        │         [Compare with expected_hash]                    │
        │                                                         │
        │   ┌─────────────────────────────────────────────────┐   │
        │   │ IF hash matches:                                │   │
        │   │   - COMMIT: set otadata to boot from slot B     │   │
        │   │   - ACK_OTA_DONE (status=OK)                    │   │
        │   │   - Reboot → runs slot B                        │   │
        │   │ ELSE:                                           │   │
        │   │   - DO NOT commit (slot A remains active)       │   │
        │   │   - NACK_OTA (error=CHECKSUM_MISMATCH)          │   │
        │   │   - Return to IDLE                              │   │
        │   └─────────────────────────────────────────────────┘   │
        │                                                         │
        ▼                                                         ▼
```

**Key principle:** The new image is **only committed** (boot partition set) **after** the checksum/hash is verified. If verification fails, the ESP32 does **not** commit and the old image remains active.

### 2.3 Rollback Support

| Scenario | Behavior |
|----------|----------|
| OTA succeeds, new app boots OK | New app can mark itself "valid"; rollback disabled. |
| OTA succeeds, new app crashes | Bootloader detects boot failure; rolls back to previous slot. |
| Checksum mismatch | No commit; old slot stays active; no reboot needed. |
| Power loss during OTA | otadata unchanged; old slot boots on power-up. |

ESP-IDF provides `esp_ota_mark_app_valid_cancel_rollback()` to confirm the new app is healthy after boot.

---

## 3. Step-by-Step Implementation Plan

### Phase 1: Partition and Build Setup (A/B Scheme, 8 MB Flash)

| Step | Task | Notes |
|------|------|-------|
| 1.1 | Create `partitions.csv` with A/B OTA | Two app slots: `ota_0` (A) and `ota_1` (B), plus `otadata`. Layout for 8 MB flash below. |
| 1.2 | Configure `platformio.ini` | `board_build.partitions = partitions.csv`; for ESP32-S3 use `board = esp32-s3-devkitc-1`. |
| 1.3 | Verify build and flash size | Build and confirm firmware fits within each OTA slot (2 MB each; current app ~310 KB). |
| 1.4 | First flash via esptool | Initial flash writes to slot A; otadata set to boot A. |

**Example `partitions.csv` (8 MB flash, per schematic):**

```csv
# Name,   Type, SubType, Offset,   Size,     Flags
nvs,      data, nvs,     0x9000,   0x4000,
otadata,  data, ota,     0xD000,   0x2000,
ota_0,    app,  ota_0,   0x10000,  0x200000,
ota_1,    app,  ota_1,   0x210000, 0x200000,
spiffs,   data, spiffs,  0x410000, 0x3F0000,
```

| Partition | Size | Notes |
|-----------|------|-------|
| `nvs` | 16 KB | NVS storage (0x4000; otadata must follow at 0xD000) |
| `otadata` | 8 KB | OTA state (which slot to boot) |
| `ota_0` (A) | **2 MB** | App slot A |
| `ota_1` (B) | **2 MB** | App slot B |
| `spiffs` | ~4 MB | Filesystem (optional) |
| **Total** | 8 MB | W25Q64JVSSIQ |

### Phase 2: Protocol Extensions

| Step | Task | Notes |
|------|------|-------|
| 2.1 | Define OTA command types | Add to GIMBAL_PROTOCOL: `CMD_OTA_START` (600), `CMD_OTA_CHUNK` (601), `CMD_OTA_END` (602), `CMD_OTA_ABORT` (603). |
| 2.2 | Define OTA response types | `ACK_OTA_STARTED` (2600), `ACK_OTA_CHUNK` (2601), `ACK_OTA_DONE` (2602), `NACK_OTA` (2603) with error codes. |
| 2.3 | Specify payload layouts | See table below. |
| 2.4 | Update GIMBAL_PROTOCOL.md | Document new types and payloads. |

**OTA Payload Layouts:**

| Command | Payload | Notes |
|---------|---------|-------|
| `CMD_OTA_START` | `total_size(4)`, `hash_type(1)`, `expected_hash(32)` | hash_type: 0=none, 1=CRC32, 2=SHA256. expected_hash: 4 or 32 bytes depending on type. |
| `CMD_OTA_CHUNK` | `offset(4)`, `length(2)`, `data(N)` | N ≤ 2048. Chunks sent sequentially. |
| `CMD_OTA_END` | (none or repeat hash) | Triggers checksum verification and conditional commit. |
| `CMD_OTA_ABORT` | (none) | Abort OTA; discard partial write; return to IDLE. |

| Response | Payload | Notes |
|----------|---------|-------|
| `ACK_OTA_STARTED` | `inactive_slot(1)`, `slot_size(4)` | Which slot (A=0, B=1) will receive the image. |
| `ACK_OTA_CHUNK` | `bytes_written(4)`, `progress_pct(1)` | Cumulative bytes; 0–100%. |
| `ACK_OTA_DONE` | `status(1)` | 0=OK (committed, rebooting). |
| `NACK_OTA` | `error_code(1)` | 1=SIZE_MISMATCH, 2=CHECKSUM_FAIL, 3=FLASH_ERROR, 4=TIMEOUT, etc. |

### Phase 3: ESP32 Firmware – OTA Receive Mode (Commit After Checksum)

| Step | Task | Notes |
|------|------|-------|
| 3.1 | Add OTA state to state machine | New state `OTA_RECEIVE`; enter via `CMD_OTA_START`. |
| 3.2 | Implement OTA_START handler | Validate params; determine inactive slot (A or B); begin `esp_ota_begin()` on that slot; store expected_hash; send `ACK_OTA_STARTED`. |
| 3.3 | Implement chunk handler | On OTA_CHUNK: call `esp_ota_write()` with data; accumulate running hash (if using SHA256); send `ACK_OTA_CHUNK` with progress. |
| 3.4 | Implement OTA_END handler | See **Commit Flow** below. |
| 3.5 | Implement OTA_ABORT | On CMD_OTA_ABORT or timeout: call `esp_ota_abort()`; discard partial image; return to IDLE. |
| 3.6 | Handle errors and NACK | Size overflow, flash error, bad offset → `NACK_OTA` with error code; abort session. |
| 3.7 | Suspend normal operations | In OTA_RECEIVE state: reject move/config commands; minimize IMU/INA polling; focus on UART. |

**Commit Flow (OTA_END handler):**

```
OTA_END received
    │
    ▼
┌──────────────────────────────────┐
│ 1. Finalize write               │  esp_ota_end()
└──────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────┐
│ 2. Compute hash of inactive slot│  Read back from flash; compute SHA256 or CRC32
└──────────────────────────────────┘
    │
    ▼
┌──────────────────────────────────┐
│ 3. Compare with expected_hash   │
└──────────────────────────────────┘
    │
    ├── Match ──────────────────────────────────────────────┐
    │                                                       ▼
    │                                      ┌────────────────────────────────┐
    │                                      │ 4a. COMMIT                     │
    │                                      │     esp_ota_set_boot_partition │
    │                                      │     → inactive slot becomes    │
    │                                      │       next boot target         │
    │                                      └────────────────────────────────┘
    │                                                       │
    │                                                       ▼
    │                                      ┌────────────────────────────────┐
    │                                      │ 5a. Send ACK_OTA_DONE (OK)     │
    │                                      │     Schedule reboot (1s delay) │
    │                                      └────────────────────────────────┘
    │
    └── Mismatch ───────────────────────────────────────────┐
                                                            ▼
                                           ┌────────────────────────────────┐
                                           │ 4b. DO NOT COMMIT              │
                                           │     Active slot unchanged      │
                                           │     Erase/discard inactive     │
                                           └────────────────────────────────┘
                                                            │
                                                            ▼
                                           ┌────────────────────────────────┐
                                           │ 5b. Send NACK_OTA              │
                                           │     error=CHECKSUM_FAIL        │
                                           │     Return to IDLE (no reboot) │
                                           └────────────────────────────────┘
```

**Post-boot validation (optional but recommended):**

After reboot into the new slot, the app should call `esp_ota_mark_app_valid_cancel_rollback()` once basic self-tests pass. If the app crashes before this, the bootloader can roll back to the previous slot.

### Phase 4: Host-Side OTA Tool ✓ IMPLEMENTED

| Step | Task | Status |
|------|------|--------|
| 4.1 | Create OTA upload script/tool | ✓ `tools/uart_ota.py` - reads firmware, sends OTA_START/CHUNK/END |
| 4.2 | Chunk size and pacing | ✓ 2KB chunks with 5ms delay between chunks |
| 4.3 | Retry and CRC verification | ✓ CRC32 hash verification, 3 retries per chunk |
| 4.4 | Progress reporting | ✓ Progress bar with percentage, speed, and byte count |
| 4.5 | Integrate with `load_new_fw` flow | ✓ `load_new_fw.bat` tries UART OTA first, falls back to manual |

**Files created:**
- `tools/uart_ota.py` - Main OTA upload tool
- `tools/uart_ota.bat` - Convenience wrapper script

**Usage:**
```bash
# Direct usage
python tools/uart_ota.py -p COM9

# With specific firmware
python tools/uart_ota.py -p COM9 -f firmware.bin

# Via batch script (tries OTA first, then manual fallback)
tools\load_new_fw.bat COM9

# Skip OTA, go direct to manual bootloader
tools\load_new_fw.bat COM9 --manual
```

**Bugs fixed during implementation:**
- CRC32 table had 2 typos (entries 111 and 245)
- OTA timeout check had underflow bug when `now` captured before `otaLastChunkTime` updated
- Chunk size reduced from 2KB to 240 bytes to fit protocol frame limits

### Phase 5: Robustness and Edge Cases

| Step | Task | Notes |
|------|------|-------|
| 5.1 | Timeout handling | OTA session timeout (e.g. 60s); abort if no chunk received. |
| 5.2 | Rollback on failure | If new app fails to boot, bootloader can roll back to previous partition (configurable). |
| 5.3 | Version / compatibility check | Optional: OTA_START payload includes min protocol version; ESP32 rejects if incompatible. |
| 5.4 | Logging | Log OTA progress and errors on host; minimal debug on ESP32 (avoid Serial flood during OTA). |

---

## 4. Validation Plan

### 4.1 Unit / Module Tests

| ID | Test | Pass Criteria |
|----|------|---------------|
| V1.1 | Partition table loads | Build succeeds; `ota_0`, `ota_1`, `otadata` present at expected offsets. |
| V1.2 | OTA command parsing | ESP32 correctly parses OTA_START (size, hash_type, expected_hash), OTA_CHUNK, OTA_END. |
| V1.3 | Chunk write to inactive slot | OTA_CHUNK writes correct bytes to inactive slot (not active). |
| V1.4 | Hash computation | ESP32 computes correct SHA256/CRC32 of written data. |
| V1.5 | Checksum match → commit | OTA_END with correct hash: `esp_ota_set_boot_partition` called; ACK_OTA_DONE sent. |
| V1.6 | Checksum mismatch → no commit | OTA_END with wrong hash: boot partition unchanged; NACK_OTA (CHECKSUM_FAIL) sent. |

### 4.2 Integration Tests

| ID | Test | Pass Criteria |
|----|------|---------------|
| V2.1 | Full OTA A→B | Running from slot A; OTA writes to B; checksum OK; commits B; reboots into B; PING works. |
| V2.2 | Full OTA B→A | Running from slot B; OTA writes to A; checksum OK; commits A; reboots into A; PING works. |
| V2.3 | A→B→A→B cycle | Perform 4 OTA cycles alternating slots; all succeed; no flash corruption. |
| V2.4 | Checksum mismatch (no commit) | Send OTA with wrong expected_hash; ESP32 sends NACK; does NOT reboot; still runs old slot. |
| V2.5 | OTA abort mid-transfer | Start OTA; send OTA_ABORT after 50% chunks; ESP32 returns to IDLE; inactive slot not committed. |
| V2.6 | OTA timeout | Start OTA; stop sending; ESP32 times out (60s); returns to IDLE; no commit. |
| V2.7 | Invalid chunk (bad offset) | Send chunk with wrong offset; ESP32 sends NACK; aborts OTA; no commit. |
| V2.8 | Oversized image | OTA_START with size > 2 MB (slot size); ESP32 rejects with NACK (SIZE_MISMATCH). |
| V2.9 | Post-boot validation | After OTA reboot, app calls `mark_valid`; subsequent crash does NOT roll back. |
| V2.10 | Rollback on crash | OTA succeeds; new app crashes before `mark_valid`; bootloader rolls back to previous slot. |

### 4.3 System / Field Tests

| ID | Test | Pass Criteria |
|----|------|---------------|
| V3.1 | OTA with servos connected | OTA during normal setup; servos remain safe; no I2C lockup. |
| V3.2 | OTA with backend connected | Backend triggers OTA; no serial conflicts; GUI shows progress. |
| V3.3 | Power loss during write | Power off mid-OTA; power on; old slot boots (no commit occurred). |
| V3.4 | Power loss after commit, before reboot | Power off after ACK_OTA_DONE; power on; new slot boots (commit persisted). |
| V3.5 | Repeated A↔B cycles (10+) | 10+ OTA cycles alternating slots; all succeed; no flash wear errors. |
| V3.6 | Baud rate stress | Run OTA at 921600 baud; no data loss; checksum always correct. |
| V3.7 | Corrupted chunk | Inject bit-flip in one chunk; checksum fails; no commit; old slot runs. |

### 4.4 Regression Tests

| ID | Test | Pass Criteria |
|----|------|---------------|
| V4.1 | Non-OTA commands during OTA | Commands other than OTA_* are rejected or queued per design. |
| V4.2 | OTA from TRACKING/CONFIG | OTA_START from TRACKING or CONFIG: either rejected (NACK) or state transitions defined. |
| V4.3 | Existing protocol unchanged | All existing commands (move, IMU, INA, config) work as before when not in OTA mode. |
| V4.4 | Manual bootloader still works | If UART OTA fails, manual BOOT+RESET + esptool still flashes successfully. |

### 4.5 Validation Execution Order

1. **Phase 1 (partitions):** V1.1  
2. **Phase 2 (protocol):** V1.2 (stub handlers)  
3. **Phase 3 (firmware):** V1.3, V1.4, V1.5, V1.6, V2.4 (checksum mismatch)  
4. **Phase 4 (host tool):** V2.1, V2.2, V2.3 (A↔B cycles)  
5. **Phase 5 (robustness):** V2.5–V2.10, V3.1–V3.7, V4.1–V4.4  

---

## 5. Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Flash corruption during OTA | Use `esp_ota_*`; write to inactive slot only; verify hash **before** commit. |
| Bad image committed | **Commit only after checksum match.** Mismatch → no commit, old slot stays active. |
| New app crashes after commit | Enable rollback; call `mark_valid` after self-test; bootloader auto-rollback if crash. |
| Power loss during write | No commit occurred; old slot boots; retry OTA. |
| Power loss after commit | Commit persisted in otadata; new slot boots. |
| UART buffer overflow | Chunk size ≤ 2 KB; pacing; optional ACK-before-next-chunk flow control. |
| Incompatible partition layout | Test build size vs slot size early; reject oversized images at OTA_START. |
| Breaking existing protocol | OTA commands additive only; no changes to existing TYPEs. |

---

## 6. Dependencies and Prerequisites

- **Hardware:** ESP32-S3-DevKitC-1 (8 MB flash W25Q64JVSSIQ, 8 MB PSRAM ESP-PSRAM64H, CH343E USB-UART)
- Arduino `Update` library (built-in) or ESP-IDF `esp_ota_ops.h`
- Partition table with at least two app slots (2 MB each for 8 MB flash)
- Host: Python, pyserial, and binary frame encode/decode (can reuse backend `protocol.py`)

---

## 7. Deliverables (Summary)

| Deliverable | Description |
|-------------|-------------|
| `partitions.csv` | A/B OTA partition layout (`ota_0`, `ota_1`, `otadata`) |
| Protocol extensions | OTA command/response types in `GIMBAL_PROTOCOL.md` |
| Firmware changes | OTA state, handlers, commit-after-checksum logic in `.ino` |
| Host OTA tool | `tools/uart_ota.py` – sends firmware, computes hash, shows progress |
| Validation report | Results of V1.x–V4.x tests |

---

## 8. Hardware Reference (from Schematic)

| Component | Part | Spec |
|-----------|------|------|
| MCU module | ESP32-S3-WROOM-1-N4R2 | Dual-core 240 MHz, WiFi + BLE |
| Flash | W25Q64JVSSIQ | 64 Mbit = **8 MB** SPI |
| PSRAM | ESP-PSRAM64H | 64 Mbit = **8 MB** octal |
| USB–UART | CH343E | USB to serial bridge |
| Power | AMS1117-3.3, etc. | 3.3 V |
| I/O | BOOT, RST | Boot and reset buttons |

---

*Document version: 1.2 | 8 MB flash (schematic), A/B images, commit-after-checksum | No code changes in this plan*

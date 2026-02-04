# Pan-Tilt Controller - Design Documentation

## Navigation

| Document | Description |
|----------|-------------|
| [Package Overview](packages.md) | Module hierarchy and dependencies |
| **Firmware** | |
| [Main Controller](firmware/main-controller.md) | ESP32 main loop and state machine |
| [Servo Control](firmware/servo-control.md) | ST3215 servo bus interface |
| [IMU Module](firmware/imu-module.md) | QMI8658 + AK09918 sensor fusion |
| [Power Monitor](firmware/power-monitor.md) | INA219 power tracking |
| [OTA Handler](firmware/ota-handler.md) | A/B partition firmware updates |
| **Backend** | |
| [Flask Server](backend/flask-server.md) | Web server and SocketIO handlers |
| [Serial Bridge](backend/serial-bridge.md) | Serial communication thread |
| [Protocol Encoder](backend/protocol-encoder.md) | Binary frame encoding/decoding |
| **Protocol** | |
| [Binary Protocol](protocol/binary-protocol.md) | Frame format and CRC8 |
| [Command Reference](protocol/command-reference.md) | All command types |
| [Response Reference](protocol/response-reference.md) | All response types |
| **Hardware** | |
| [Hardware Interfaces](hardware/interfaces.md) | Pin mappings and connections |
| **Tools** | |
| [OTA Uploader](tools/ota-uploader.md) | UART OTA firmware tool |

---

## High-Level Design (HLD)

### 1. System Overview

The Pan-Tilt Controller is a custom firmware and tooling system for the **Waveshare 2-Axis Pan-Tilt Camera Module** using an **ESP32-S3** microcontroller and **ST3215 serial bus servos**.

```mermaid
graph TB
    subgraph Host["Host PC"]
        WebGUI[Web GUI<br/>Browser]
        Backend[Flask Backend<br/>Python]
        OTATool[OTA Tool<br/>Python]
    end

    subgraph ESP32["ESP32-S3 Controller"]
        FW[Firmware<br/>C++/Arduino]
        OTA[OTA Handler]
        IMU[IMU Driver]
        PWR[Power Monitor]
        SERVO[Servo Driver]
    end

    subgraph Hardware["Hardware"]
        PAN[Pan Servo<br/>ST3215]
        TILT[Tilt Servo<br/>ST3215]
        QMI[QMI8658C<br/>Accel/Gyro]
        AK[AK09918C<br/>Magnetometer]
        INA[INA219<br/>Power]
    end

    WebGUI <-->|WebSocket| Backend
    Backend <-->|Serial 921600| FW
    OTATool -->|Serial 921600| OTA

    FW --> SERVO
    SERVO <-->|UART1 1Mbps| PAN
    SERVO <-->|UART1 1Mbps| TILT

    FW --> IMU
    IMU <-->|I2C| QMI
    IMU <-->|I2C| AK

    FW --> PWR
    PWR <-->|I2C| INA
```

### 2. Architecture Layers

```mermaid
graph TB
    subgraph Application["Application Layer"]
        WEB[Web GUI]
        CLI[CLI Tools]
    end

    subgraph Transport["Transport Layer"]
        SOCK[SocketIO]
        SER[Serial Bridge]
    end

    subgraph Protocol["Protocol Layer"]
        BIN[Binary Protocol<br/>CRC8 Frames]
    end

    subgraph Firmware["Firmware Layer"]
        SM[State Machine]
        CMD[Command Processor]
    end

    subgraph Drivers["Driver Layer"]
        SDRV[Servo Driver<br/>SCServo]
        IDRV[IMU Driver<br/>AHRS Fusion]
        PDRV[Power Driver<br/>INA219]
    end

    subgraph HAL["Hardware Abstraction"]
        UART0[UART0<br/>USB-Host]
        UART1[UART1<br/>Servo Bus]
        I2C[I2C Bus<br/>Sensors]
    end

    Application --> Transport
    Transport --> Protocol
    Protocol --> Firmware
    Firmware --> Drivers
    Drivers --> HAL
```

### 3. Key Features

| Feature | Description |
|---------|-------------|
| **Binary Protocol** | Custom UART protocol at 921600 baud with CRC8 checksums |
| **OTA Updates** | A/B partition scheme with hash verification before commit |
| **Real-time Control** | 100 Hz IMU fusion, 20 Hz servo feedback |
| **State Machine** | IDLE, TRACKING, CONFIG, OTA_RECEIVE states |
| **Safety Watchdog** | Heartbeat timeout and auto-lock mechanisms |
| **Web GUI** | Real-time control via Flask + SocketIO |

### 4. Data Flow

```mermaid
sequenceDiagram
    participant GUI as Web GUI
    participant BE as Backend
    participant ESP as ESP32
    participant SRV as Servos
    participant IMU as IMU

    GUI->>BE: Move command (WebSocket)
    BE->>ESP: Binary frame (Serial)
    ESP->>SRV: SyncWritePosEx (UART1)
    SRV-->>ESP: Position feedback
    ESP-->>BE: ACK_EXECUTED + feedback
    BE-->>GUI: Status update (WebSocket)

    loop Every 10ms
        IMU-->>ESP: Sensor data (I2C)
        ESP->>ESP: AHRS fusion
    end

    loop Periodic (100ms default)
        ESP-->>BE: IMU/INA data
        BE-->>GUI: Sensor display
    end
```

### 5. State Machine

```mermaid
stateDiagram-v2
    [*] --> IDLE: Power On

    IDLE --> TRACKING: CMD_ENTER_TRACKING
    IDLE --> CONFIG: CMD_ENTER_CONFIG
    IDLE --> OTA_RECEIVE: CMD_OTA_START

    TRACKING --> IDLE: CMD_PAN_TILT_STOP
    TRACKING --> TRACKING: Move Commands

    CONFIG --> IDLE: CMD_EXIT_CONFIG
    CONFIG --> CONFIG: Servo Test Commands

    OTA_RECEIVE --> IDLE: OTA_END (success)
    OTA_RECEIVE --> IDLE: OTA_ABORT
    OTA_RECEIVE --> IDLE: Timeout (60s)
    OTA_RECEIVE --> OTA_RECEIVE: OTA_CHUNK
```

### 6. Communication Interfaces

| Interface | Speed | Protocol | Purpose |
|-----------|-------|----------|---------|
| USB-UART (UART0) | 921600 baud | Binary frames | Host communication |
| Servo Bus (UART1) | 1 Mbps | Half-duplex TTL | ST3215 control |
| I2C | 400 kHz | Standard I2C | IMU + Power monitor |

### 7. Update Rates

| Component | Rate | Period |
|-----------|------|--------|
| IMU Sensor Read | 100 Hz | 10 ms |
| AHRS Fusion | 100 Hz | 10 ms |
| Power Monitor | 20 Hz | 50 ms |
| Servo Feedback | 20 Hz | 50 ms |
| Periodic Response | 10 Hz | 100 ms (default) |

### 8. Safety Mechanisms

```mermaid
graph LR
    subgraph Watchdogs["Safety Watchdogs"]
        HB[Heartbeat<br/>Configurable timeout]
        AL[Auto-Lock<br/>500ms idle]
        OT[OTA Timeout<br/>60 sec]
    end

    subgraph Actions["Safety Actions"]
        STOP[Stop Motion]
        LOCK[Lock Position]
        ABORT[Abort OTA]
    end

    HB -->|Timeout| STOP
    AL -->|Idle| LOCK
    OT -->|No chunks| ABORT
```

---

## Technology Stack

| Layer | Technology |
|-------|------------|
| Microcontroller | ESP32-S3 (dual-core 240 MHz) |
| Firmware | Arduino/PlatformIO (C++) |
| Backend | Python 3, Flask, SocketIO |
| Protocol | Custom binary, CRC8 |
| Servos | ST3215 (SCServo library) |
| IMU | QMI8658C + AK09918C |
| Power | INA219 |
| OTA | ESP-IDF OTA API, A/B partitions |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2024 | Initial release with basic control |
| 1.1 | 2024 | Added OTA support, A/B scheme |
| 1.2 | 2024 | TRACKING mode optimization |

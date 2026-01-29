# Pan-Tilt Program Deep Review

**Firmware Version:** 0.9  
**Hardware:** Waveshare General Driver for Robots (ESP32-D0WD-V3)  
**Module Type:** Pan-Tilt Gimbal (moduleType=2)  
**Main Type:** WAVE ROVER (mainType=1)

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Program Flow](#program-flow)
3. [Gimbal Control System](#gimbal-control-system)
4. [Communication Protocols](#communication-protocols)
5. [Hardware Interfaces](#hardware-interfaces)
6. [Control Algorithms](#control-algorithms)
7. [Configuration System](#configuration-system)
8. [Data Structures](#data-structures)
9. [API Reference](#api-reference)
10. [Code Quality Analysis](#code-quality-analysis)

---

## System Architecture

### Overview

The pan-tilt firmware is a modular ESP32 application that controls a 2-axis gimbal camera system. It uses ST3215 serial bus servos for pan/tilt control, integrates IMU sensors for stabilization, and provides multiple communication interfaces (USB Serial, WiFi Web, ESP-NOW).

### Module Organization

```
pan_tilt_base_v0.9/
├── pan_tilt_base_v0.9.ino    # Main program entry point
├── gimbal_module.h            # Gimbal control functions
├── json_cmd.h                 # JSON command definitions
├── uart_ctrl.h                # Serial command handler
├── http_server.h              # Web server interface
├── wifi_ctrl.h                # WiFi management
├── esp_now_ctrl.h             # ESP-NOW wireless communication
├── IMU_ctrl.h                 # IMU sensor control
├── oled_ctrl.h                # OLED display control
├── battery_ctrl.h              # Power monitoring
├── files_ctrl.h               # LittleFS file operations
├── movtion_module.h           # Motion control (motors/encoders)
├── ugv_config.h               # Configuration constants
├── ugv_advance.h              # Mission system
├── ugv_led_ctrl.h             # LED control
├── RoArm-M2_module.h          # Robot arm control (not used in PT mode)
└── data/
    ├── devConfig.json         # Device configuration
    └── wifiConfig.json        # WiFi settings
```

### Key Libraries

- **SCServo**: ST3215 serial bus servo control
- **ArduinoJson**: JSON parsing and generation
- **WiFi/WebServer**: WiFi and HTTP server
- **esp_now**: ESP-NOW peer-to-peer communication
- **Adafruit_SSD1306**: OLED display driver
- **INA219_WE**: Power monitoring
- **Adafruit_ICM20X/ICM20948**: IMU sensors
- **ESP32Encoder**: Encoder reading
- **PID_v2**: PID control for motors
- **SimpleKalmanFilter**: IMU data filtering
- **LittleFS**: File system for configuration storage

---

## Program Flow

### Initialization Sequence (setup())

**File:** `pan_tilt_base_v0.9.ino` (lines 100-222)

The initialization follows this strict sequence:

1. **Serial Communication**
   - Initialize Serial at 115200 baud
   - Wait for serial connection

2. **I2C Bus**
   - Initialize Wire library (SDA: GPIO 32, SCL: GPIO 33)

3. **Battery Monitor**
   - Initialize INA219 (address 0x42)
   - First voltage reading

4. **Module Configuration**
   - Set mainType = 1 (WAVE ROVER)
   - Set moduleType = 2 (Pan-Tilt/Gimbal)
   - Call `mm_settings()` to configure system parameters

5. **OLED Display**
   - Initialize SSD1306 (128x32, I2C 0x3C)
   - Display "PT CAM" and "version: 0.90"
   - Update status messages during initialization

6. **IMU Sensors**
   - Initialize QMI8658 (6-axis: accel + gyro)
   - Initialize AK09918C (3-axis magnetometer)

7. **LED Control**
   - Initialize PWM channels for LED control
   - GPIO 4 (channel 7) and GPIO 5 (channel 8)

8. **File System**
   - Initialize LittleFS
   - Mount filesystem for configuration storage

9. **Motion Pins**
   - Initialize GPIO pins for 12V switch control
   - Motor control pins (if applicable)

10. **Servo System**
    - Power up servos (500ms delay)
    - Initialize UART2 for servo communication (GPIO 18/19)
    - Check servo status (PAN ID=2, TILT ID=1)
    - Move to initial position
    - Set torque limits

11. **Network Services**
    - Initialize WiFi (AP/STA/AP+STA mode)
    - Initialize HTTP web server (port 80)
    - Initialize ESP-NOW

12. **Control Systems**
    - Initialize encoders (for motor control)
    - Initialize PID controllers
    - Get device MAC address
    - Update OLED with WiFi info

13. **Boot Mission**
    - Execute "boot" mission if it exists
    - Runs automatically on startup

### Main Loop Structure (loop())

**File:** `pan_tilt_base_v0.9.ino` (lines 225-261)

The main loop executes these functions in sequence:

```cpp
void loop() {
  serialCtrl();              // Parse JSON from USB serial
  server.handleClient();     // Handle HTTP web requests
  
  // Module-specific control loop
  switch (moduleType) {
    case 1: moduleType_RoArmM2(); break;  // Robot arm mode
    case 2: moduleType_Gimbal(); break;   // Gimbal mode (active)
  }
  
  // Process received JSON commands
  if(runNewJsonCmd) {
    jsonCmdReceiveHandler();
    jsonCmdReceive.clear();
    runNewJsonCmd = false;
  }
  
  // Motor control (for UGV base)
  getLeftSpeed();
  LeftPidControllerCompute();
  getRightSpeed();
  RightPidControllerCompute();
  
  // Status updates
  oledInfoUpdate();          // Update OLED every 10 seconds
  updateIMUData();           // Read IMU sensors
  
  // Feedback
  if (baseFeedbackFlow) {
    baseInfoFeedback();      // Send base info if enabled
  }
  
  heartBeatCtrl();           // Heartbeat/timeout control
}
```

**Timing:**
- Loop runs continuously (no delays)
- OLED updates every 10 seconds
- Servo feedback read every loop iteration
- IMU data updated every loop iteration

### Gimbal Module Loop

**Function:** `moduleType_Gimbal()` (lines 94-97)

```cpp
void moduleType_Gimbal() {
  getGimbalFeedback();       // Read servo positions/status
  gimbalSteady(steadyGoalY); // Apply IMU stabilization if enabled
}
```

**Key Operations:**
1. **Servo Feedback**: Reads position, speed, load, voltage, current, temperature from both servos
2. **IMU Stabilization**: If `steadyMode` is enabled, compensates tilt axis for pitch angle

---

## Gimbal Control System

### Servo Configuration

**Servo IDs:**
- **PAN (Horizontal)**: ID = 2 (`GIMBAL_PAN_ID`)
- **TILT (Vertical)**: ID = 1 (`GIMBAL_TILT_ID`)

**Servo Type:** ST3215 serial bus servo
- **Position Range:** 0-4095 (12-bit resolution)
- **Center Position:** 2047
- **Angle Range:** 360° per servo rotation
- **Communication:** UART2 (GPIO 18 RX, GPIO 19 TX)
- **Protocol:** SCServo library (ST3215 protocol)

### Control Functions

#### 1. Simple Position Control

**Function:** `gimbalCtrlSimple(float Xinput, float Yinput, float spdInput, float accInput)`

**Parameters:**
- `Xinput`: Pan angle in degrees (-180° to 180°)
- `Yinput`: Tilt angle in degrees (-30° to 90°)
- `spdInput`: Speed (0 = fastest, higher = slower)
- `accInput`: Acceleration (0 = fastest, higher = slower)

**Algorithm:**
```cpp
// Constrain inputs
Xinput = constrainFloat(Xinput, -180, 180);
Yinput = constrainFloat(Yinput, -30, 90);

// Convert angles to servo positions
gimbalPos[0] = 2047 + round(map(Xinput, 0, 360, 0, 4095));  // PAN
gimbalPos[1] = 2047 - round(map(Yinput, 0, 360, 0, 4095));  // TILT (inverted)

// Convert speed/acceleration
gimbalSpd[0] = round(map(spdInput, 0, 360, 0, 4095));
gimbalSpd[1] = round(map(spdInput, 0, 360, 0, 4095));
gimbalAcc[0] = round(map(accInput, 0, 360, 0, 4095));
gimbalAcc[1] = round(map(accInput, 0, 360, 0, 4095));

// Send synchronized command to both servos
st.SyncWritePosEx(gimbalID, 2, gimbalPos, gimbalSpd, gimbalAcc);
```

**Usage:** Absolute positioning with synchronized movement

#### 2. Continuous Move Control

**Function:** `gimbalCtrlMove(float Xinput, float Yinput, float spdInputX, float spdInputY)`

**Parameters:**
- `Xinput`: Target pan angle (-180° to 180°)
- `Yinput`: Target tilt angle (-30° to 90°)
- `spdInputX`: Pan speed (1-2500)
- `spdInputY`: Tilt speed (1-2500)

**Algorithm:**
- Similar to `gimbalCtrlSimple()` but with separate speeds for each axis
- Acceleration set to 0 for continuous movement

**Usage:** Continuous movement with independent axis speeds

#### 3. Emergency Stop

**Function:** `gimbalCtrlStop()`

**Algorithm:**
```cpp
st.EnableTorque(GIMBAL_PAN_ID, 0);   // Disable torque
st.EnableTorque(GIMBAL_TILT_ID, 0);
delay(SERVO_STOP_DELAY);              // Wait 3ms
st.EnableTorque(GIMBAL_PAN_ID, 1);   // Re-enable torque
st.EnableTorque(GIMBAL_TILT_ID, 1);
```

**Usage:** Emergency stop - releases and re-engages servos

#### 4. IMU Stabilization (Steady Mode)

**Function:** `gimbalSteady(float inputBiasY)`

**Algorithm:**
```cpp
if (!steadyMode) {
    return;  // Steady mode disabled
}
// Compensate tilt axis for IMU pitch
gimbalCtrlSimple(0, inputBiasY - icm_pitch, 0, 0);
```

**How it works:**
- Reads IMU pitch angle (`icm_pitch`)
- Subtracts pitch from target Y angle
- Keeps camera level when base tilts
- Only affects Y (tilt) axis
- X (pan) axis remains at 0

**Control:**
- Enable: `{"T":137,"s":1,"y":0}` - `s=1` enables, `y` is target angle
- Disable: `{"T":137,"s":0,"y":0}` - `s=0` disables

**Example:** If base tilts 10° forward, tilt servo compensates by -10° to keep camera level

#### 5. User Control Mode

**Function:** `gimbalUserCtrl(int inputX, int inputY, int inputSpd)`

**Parameters:**
- `inputX`: -1 (left), 0 (stop), 1 (right), 2 (center)
- `inputY`: -1 (down), 0 (stop), 1 (up), 2 (center)
- `inputSpd`: Speed value

**Algorithm:**
- Maintains goal positions (`goalX`, `goalY`)
- Updates goal based on input direction
- If input is 0, releases torque briefly to read current position
- Moves toward goal at specified speed

**Usage:** UI control with incremental movement

#### 6. Shell Control Mode

**Function:** `gimbalUserCtrlShell(float lSpd, float rSpd)`

**Algorithm:**
- Converts motor speeds (left/right) to gimbal commands
- Used for joystick/analog control
- Maps speed differences to pan/tilt directions

**Usage:** Convert motor control signals to gimbal movement

### Position Feedback

**Function:** `getGimbalFeedback()`

**Reads from each servo:**
- Position (0-4095)
- Speed
- Load (torque)
- Voltage
- Current
- Temperature
- Mode

**Error Handling:**
- If servo doesn't respond, sets `status = false`
- Sends error JSON: `{"T":1005,"id":X,"status":0}`

### Angle Conversion

**Functions:**
- `panAngleCompute(int inputPos)`: Converts servo position to pan angle (0-360°)
- `tiltAngleCompute(int inputPos)`: Converts servo position to tilt angle (0-360°, inverted)

**Formulas:**
```cpp
panAngle = mapFloat((inputPos - 2047), 0, 4095, 0, 360);
tiltAngle = mapFloat((2047 - inputPos), 0, 4095, 0, 360);
```

---

## Communication Protocols

### 1. USB Serial (UART)

**Interface:** USB Type-C port (CP2102 USB-to-UART)

**Protocol:** JSON commands over serial line

**Function:** `serialCtrl()` in `uart_ctrl.h`

**Format:**
- Commands sent as JSON strings
- Terminated by newline (`\n`)
- Responses sent as JSON strings

**Example:**
```json
{"T":133,"X":45,"Y":10,"SPD":0,"ACC":0}\n
```

**Handler:** `jsonCmdReceiveHandler()` processes command type `T` and routes to appropriate function

### 2. HTTP Web Server

**Port:** 80

**Endpoints:**
- `GET /` - Serves HTML web interface (`index_html` from `web_page.h`)
- `GET /js?{JSON}` - Receives JSON command, executes, returns feedback

**Function:** `initHttpWebServer()` in `http_server.h`

**Web Interface:**
- HTML page with control buttons
- Sends JSON commands via `/js` endpoint
- Receives feedback as JSON response

**Example Request:**
```
GET /js?{"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}
```

**Response:**
```json
{"T":1001,"info":"command executed"}
```

### 3. ESP-NOW

**Protocol:** ESP-NOW peer-to-peer wireless communication

**Modes:**
- **0**: None (disabled)
- **1**: Flow-leader (group) - Sends commands to multiple followers
- **2**: Flow-leader (single) - Sends commands to one follower
- **3**: Follower (default) - Receives commands

**Function:** `initEspNow()` in `esp_now_ctrl.h`

**Features:**
- Broadcast control (MAC: FF:FF:FF:FF:FF:FF)
- MAC address whitelist
- Peer management (add/remove followers)
- Status callbacks (send success/failure)

**Message Structure:**
```cpp
struct struct_message {
  byte devCode;
  float base;
  float shoulder;
  float elbow;
  float hand;
  byte cmd;
  char message[210];  // JSON command string
}
```

**Usage:** Wireless control of multiple devices simultaneously

### 4. JSON Command Protocol

**All communication uses JSON format with command type `T`**

**Command Structure:**
```json
{
  "T": <command_type>,
  <parameter1>: <value1>,
  <parameter2>: <value2>,
  ...
}
```

**Response Structure:**
```json
{
  "T": <feedback_type>,
  <data_field1>: <value1>,
  ...
}
```

**Command Routing:**
- `jsonCmdReceiveHandler()` reads `T` field
- Routes to appropriate handler via switch statement
- All handlers in `uart_ctrl.h`

---

## Hardware Interfaces

### Servo Control (ST3215 Serial Bus Servos)

**UART Interface:**
- **RX:** GPIO 18 (`S_RXD`)
- **TX:** GPIO 19 (`S_TXD`)
- **Baud Rate:** Set by SCServo library (typically 1000000 bps)
- **Protocol:** ST3215 serial bus protocol

**Library:** SCServo (`SMS_STS` class)

**Key Functions:**
- `st.SyncWritePosEx()` - Synchronized position control
- `st.FeedBack()` - Read servo feedback
- `st.EnableTorque()` - Enable/disable torque
- `st.ReadPos()` - Read position
- `st.ReadSpeed()` - Read speed
- `st.ReadLoad()` - Read load/torque
- `st.ReadVoltage()` - Read voltage
- `st.ReadCurrent()` - Read current
- `st.ReadTemper()` - Read temperature
- `st.ReadMode()` - Read servo mode

**Servo Feedback Structure:**
```cpp
struct ServoFeedback {
  bool status;      // Communication success
  int pos;          // Position (0-4095)
  int speed;        // Current speed
  int load;         // Load/torque
  float voltage;    // Voltage (V)
  float current;    // Current (mA)
  float temper;     // Temperature (°C)
  byte mode;        // Servo mode
};
```

### IMU Sensors

**Sensors:**
- **QMI8658**: 6-axis motion sensor (accelerometer + gyroscope)
- **AK09918C**: 3-axis magnetometer

**I2C Interface:**
- **SDA:** GPIO 32 (`S_SDA`)
- **SCL:** GPIO 33 (`S_SCL`)
- **Address:** Detected automatically

**Library:** Custom IMU library (`IMU.h`, `QMI8658.cpp`, `AK09918.cpp`)

**Data Output:**
```cpp
// Euler angles (degrees)
double icm_roll;   // Roll angle
double icm_pitch;  // Pitch angle (used for stabilization)
double icm_yaw;    // Yaw angle

// Raw sensor data
double ax, ay, az;  // Accelerometer (m/s²)
double gx, gy, gz;  // Gyroscope (rad/s)
double mx, my, mz;  // Magnetometer (raw counts)
```

**Update Rate:** Every loop iteration (no filtering delay)

**Function:** `updateIMUData()` in `IMU_ctrl.h`

### OLED Display

**Display:** SSD1306 128x32 monochrome OLED

**I2C Interface:**
- **Address:** 0x3C
- **Same I2C bus** as IMU sensors

**Library:** Adafruit_SSD1306

**Display Format:**
- 4 lines of text (8 pixels per line)
- Line 0: Module name / WiFi AP SSID
- Line 1: Version / WiFi STA IP
- Line 2: Status messages / Device type
- Line 3: Battery voltage / ESP-NOW status

**Update:**
- Status messages: Immediate (during setup)
- Battery voltage: Every 10 seconds
- WiFi info: On connection change

**Function:** `oled_update()` in `oled_ctrl.h`

### Power Monitoring

**Sensor:** INA219 (I2C address 0x42)

**Library:** INA219_WE

**Measurements:**
```cpp
float shuntVoltage_mV;  // Shunt voltage
float loadVoltage_V;    // Load voltage (bus + shunt)
float busVoltage_V;     // Bus voltage
float current_mA;       // Current consumption
float power_mW;         // Power consumption
bool ina219_overflow;   // Overflow flag
```

**Configuration:**
- ADC mode: 9-bit
- PGA gain: 320mV
- Bus range: 16V
- Shunt resistor: 0.01Ω

**Update Rate:** Every 10 seconds (in `oledInfoUpdate()`)

**Function:** `inaDataUpdate()` in `battery_ctrl.h`

### LED Control

**Pins:**
- **IO4:** GPIO 4 (PWM channel 7)
- **IO5:** GPIO 5 (PWM channel 8)

**PWM Settings:**
- Frequency: 200 Hz
- Resolution: 8-bit (0-255)
- Range: 0 (off) to 255 (max brightness)

**Function:** `led_pwm_ctrl(int io4Input, int io5Input)`

**Note:** In pan-tilt mode, LED control is repurposed for motor control:
```cpp
leftCtrl(-abs(io5Input));
rightCtrl(-abs(io4Input));
```

### Motor Control (For UGV Base)

**Pins:**
- **Motor A:** PWMA=25, AIN1=21, AIN2=17
- **Motor B:** PWMB=26, BIN1=22, BIN2=23
- **Encoders:** AENCA=35, AENCB=34, BENCA=27, BENCB=16

**Control:** TB6612FNG motor driver chip

**Note:** Motors are not used in pan-tilt-only mode, but PID controllers still run

---

## Control Algorithms

### Gimbal Position Control

**Algorithm Flow:**

1. **Input Validation**
   ```cpp
   Xinput = constrainFloat(Xinput, -180, 180);
   Yinput = constrainFloat(Yinput, -30, 90);
   ```

2. **Angle to Position Conversion**
   ```cpp
   // Pan: 0° = 2047, +180° = 4095, -180° = 0
   gimbalPos[0] = 2047 + round(map(Xinput, 0, 360, 0, 4095));
   
   // Tilt: 0° = 2047, +90° = 0, -30° = 4095 (inverted)
   gimbalPos[1] = 2047 - round(map(Yinput, 0, 360, 0, 4095));
   ```

3. **Speed/Acceleration Mapping**
   - Speed: 0 = fastest, higher = slower
   - Acceleration: 0 = fastest, higher = slower
   - Mapped to 0-4095 range

4. **Synchronized Command**
   - Sends command to both servos simultaneously
   - Ensures coordinated movement

### IMU Stabilization Algorithm

**Steady Mode Logic:**

```cpp
void gimbalSteady(float inputBiasY) {
  if (!steadyMode) return;
  
  // Compensate for base pitch
  float targetY = inputBiasY - icm_pitch;
  
  // Apply compensation (pan stays at 0)
  gimbalCtrlSimple(0, targetY, 0, 0);
}
```

**How it works:**
1. Read IMU pitch angle (`icm_pitch`)
2. Subtract pitch from target Y angle
3. If base tilts forward (+pitch), tilt servo moves backward (-pitch)
4. Keeps camera level relative to gravity

**Limitations:**
- Only compensates pitch (vertical tilt)
- Pan axis not affected
- No roll compensation
- Requires IMU calibration

### User Control Algorithm

**State Machine:**

```cpp
static float goalX = 0;  // Maintained between calls
static float goalY = 0;

// Update goal based on input
if (inputX == -1) goalX = -180;  // Left
if (inputX == 1)  goalX = 180;   // Right
if (inputX == 0) {
    // Read current position
    servoTorqueCtrl(GIMBAL_PAN_ID, 0);
    delay(5);
    servoTorqueCtrl(GIMBAL_PAN_ID, 1);
    getGimbalFeedback();
    goalX = panAngleCompute(gimbalFeedback[0].pos);
}
// Similar for Y axis
```

**Features:**
- Maintains goal positions
- Reads actual position when input is 0 (stop)
- Prevents drift by re-reading position

### Motor Speed to Gimbal Conversion

**Function:** `gimbalUserCtrlShell(float lSpd, float rSpd)`

**Logic:**
- Both speeds positive and equal → Move up (Y=1)
- Both speeds negative and equal → Move down (Y=-1)
- Left negative, right positive → Move left (X=-1)
- Left positive, right negative → Move right (X=1)
- Different magnitudes → Diagonal movement

**Usage:** Convert joystick/analog stick to gimbal control

---

## Configuration System

### Module Types

**Main Type (`mainType`):**
- **1**: WAVE ROVER (current)
- **2**: UGV02
- **3**: UGV01

**Module Type (`moduleType`):**
- **0**: None (base only)
- **1**: RoArm-M2 (robot arm)
- **2**: Pan-Tilt/Gimbal (current)

**Configuration Function:** `mm_settings(byte inputMain, byte inputModule)`

**Effects:**
- Sets wheel diameter, encoder pulses, track width for UGV
- Configures PID compute mode
- Updates OLED display text

### WiFi Configuration

**Modes:**
- **0**: OFF
- **1**: AP (Access Point) - Creates hotspot
- **2**: STA (Station) - Connects to WiFi
- **3**: AP+STA - Both modes simultaneously

**Default Settings:**
- **AP SSID:** "PT" (for pan-tilt)
- **AP Password:** "12345678"
- **Boot Mode:** 1 (AP mode by default)

**Configuration File:** `data/wifiConfig.json`

**Format:**
```json
{
  "wifi_mode_on_boot": 3,
  "sta_ssid": "YourWiFi",
  "sta_password": "password",
  "ap_ssid": "PT",
  "ap_password": "12345678"
}
```

**Functions:**
- `initWifi()` - Initialize WiFi on boot
- `loadWifiConfig()` - Load from LittleFS
- `wifiModeAP()` - Switch to AP mode
- `wifiModeSTA()` - Switch to STA mode
- `wifiModeAPSTA()` - Switch to AP+STA mode

### Device Configuration

**File:** `data/devConfig.json`

**Purpose:** Device-specific settings stored in flash

**Management:** Via JSON commands (T=200-208) for file operations

### Mission System

**Purpose:** Store and execute sequences of commands

**File Format:** `.mission` files in LittleFS

**Structure:**
- First line: Mission metadata (JSON with name and intro)
- Subsequent lines: JSON commands (one per line)

**Example Mission:**
```
{"name":"boot","intro":"these cmds run automatically at boot."}
{"T":2,"P":20,"I":2500,"D":0,"L":255}
{"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}
{"T":111,"cmd":1000}
```

**Functions:**
- `createMission()` - Create new mission file
- `missionContent()` - Read mission steps
- `appendStepJson()` - Add step to end
- `insertStepJson()` - Insert step at position
- `replaceStepJson()` - Replace step
- `deleteStep()` - Delete step
- `missionPlay()` - Execute mission (with repeat count)

**Boot Mission:**
- Automatically executed on startup
- Name: "boot"
- Created in `setup()` if doesn't exist

---

## Data Structures

### Gimbal Control Variables

```cpp
// Servo IDs
u8 gimbalID[2] = {GIMBAL_PAN_ID, GIMBAL_TILT_ID};  // {2, 1}

// Target positions (servo units: 0-4095)
s16 gimbalPos[2];  // [0]=PAN, [1]=TILT

// Movement parameters
u16 gimbalSpd[2];  // Speed
u8  gimbalAcc[2];  // Acceleration

// Feedback data
ServoFeedback gimbalFeedback[2];  // [0]=PAN, [1]=TILT

// Steady mode
float steadyGoalY = 0;  // Target Y angle for steady mode
bool steadyMode = false; // Steady mode enabled
```

### IMU Data

```cpp
// Euler angles (degrees)
double icm_roll;   // Roll
double icm_pitch;  // Pitch (used for stabilization)
double icm_yaw;    // Yaw
double icm_temp;   // Temperature

// Raw sensor data
double ax, ay, az;  // Accelerometer (m/s²)
double gx, gy, gz;  // Gyroscope (rad/s)
double mx, my, mz;  // Magnetometer (raw)

// Quaternion (if available)
double qw, qx, qy, qz;
```

### JSON Documents

```cpp
StaticJsonDocument<256> jsonCmdReceive;  // Received commands
StaticJsonDocument<256> jsonInfoSend;     // Sent info
StaticJsonDocument<512> jsonInfoHttp;    // HTTP responses
```

### System State

```cpp
byte mainType = 1;        // 1=WAVE ROVER
byte moduleType = 2;      // 2=Pan-Tilt
bool steadyMode = false;  // IMU stabilization
bool baseFeedbackFlow = 1; // Enable base info feedback
byte InfoPrint = 2;       // Debug print level
byte espNowMode = 3;      // ESP-NOW mode (3=follower)
```

---

## API Reference

### Complete JSON Command Set

#### Gimbal Control Commands

##### CMD_GIMBAL_CTRL_SIMPLE (T=133)

**Description:** Absolute position control with speed and acceleration

**Command:**
```json
{"T":133,"X":45,"Y":10,"SPD":0,"ACC":0}
```

**Parameters:**
- `X`: Pan angle in degrees (-180 to 180)
- `Y`: Tilt angle in degrees (-30 to 90)
- `SPD`: Speed (0=fastest, higher=slower)
- `ACC`: Acceleration (0=fastest, higher=slower)

**Function:** `gimbalCtrlSimple()` in `gimbal_module.h`

**Example:**
```json
{"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}  // Center gimbal at max speed
{"T":133,"X":90,"Y":45,"SPD":100,"ACC":50}  // Move to position with speed/accel
```

**Response:** None (command executed)

##### CMD_GIMBAL_CTRL_MOVE (T=134)

**Description:** Continuous movement with separate X/Y speeds

**Command:**
```json
{"T":134,"X":45,"Y":45,"SX":300,"SY":300}
```

**Parameters:**
- `X`: Target pan angle (-180 to 180)
- `Y`: Target tilt angle (-30 to 90)
- `SX`: Pan speed (1-2500)
- `SY`: Tilt speed (1-2500)

**Function:** `gimbalCtrlMove()` in `gimbal_module.h`

**Example:**
```json
{"T":134,"X":0,"Y":0,"SX":500,"SY":300}  // Move to center, different speeds
```

**Response:** None

##### CMD_GIMBAL_CTRL_STOP (T=135)

**Description:** Emergency stop - disable and re-enable servo torque

**Command:**
```json
{"T":135}
```

**Function:** `gimbalCtrlStop()` in `gimbal_module.h`

**Effect:** Immediately stops movement by releasing and re-engaging torque

**Response:** None

##### CMD_GIMBAL_STEADY (T=137)

**Description:** Enable/disable IMU-based stabilization

**Command:**
```json
{"T":137,"s":1,"y":0}  // Enable steady mode
{"T":137,"s":0,"y":0}  // Disable steady mode
```

**Parameters:**
- `s`: 0=disable, 1=enable
- `y`: Target Y angle (degrees, -45 to 90)

**Function:** `gimbalSteadySet()` in `gimbal_module.h`

**Effect:** When enabled, tilt axis automatically compensates for base pitch

**Response:** None

**Note:** Steady mode runs in `moduleType_Gimbal()` loop, calling `gimbalSteady()`

##### CMD_GIMBAL_USER_CTRL (T=141)

**Description:** UI control mode with incremental movement

**Command:**
```json
{"T":141,"X":-1,"Y":1,"SPD":300}
```

**Parameters:**
- `X`: -1 (left), 0 (stop/read), 1 (right), 2 (center)
- `Y`: -1 (down), 0 (stop/read), 1 (up), 2 (center)
- `SPD`: Speed value

**Function:** `gimbalUserCtrl()` in `gimbal_module.h`

**Example:**
```json
{"T":141,"X":1,"Y":0,"SPD":300}   // Move right
{"T":141,"X":0,"Y":1,"SPD":300}   // Move up
{"T":141,"X":2,"Y":2,"SPD":0}     // Center gimbal
```

**Response:** None

**Note:** Maintains goal positions between calls. When input is 0, reads current position.

### LED Control Commands

##### CMD_LED_CTRL (T=132)

**Description:** Control LED brightness

**Command:**
```json
{"T":132,"IO4":255,"IO5":255}
```

**Parameters:**
- `IO4`: LED brightness (0-255, GPIO 4)
- `IO5`: LED brightness (0-255, GPIO 5, used for pan-tilt LED)

**Function:** `led_pwm_ctrl()` in `ugv_led_ctrl.h`

**Note:** In pan-tilt mode, also controls motor direction (repurposed)

**Response:** None

### System Control Commands

##### CMD_TORQUE_CTRL (T=210)

**Description:** Enable/disable servo torque lock

**Command:**
```json
{"T":210,"cmd":0}  // Disable torque (release)
{"T":210,"cmd":1}  // Enable torque (lock)
```

**Function:** `servoTorqueCtrl()` in `RoArm-M2_module.h`

**Note:** Uses broadcast ID 254 to control all servos simultaneously

**Usage:** Disable torque for manual positioning, then re-enable

**Response:** None (device reboots)

##### CMD_REBOOT (T=600)

**Description:** Reboot ESP32

**Command:**
```json
{"T":600}
```

**Function:** `esp_restart()` - ESP32 system function

**Response:** Device reboots (no response)

### Feedback Commands

##### CMD_BASE_FEEDBACK (T=130)

**Description:** Get base information feedback

**Command:**
```json
{"T":130}
```

**Function:** `baseInfoFeedback()` in `ugv_advance.h`

**Response:** JSON with device status, IMU data, servo feedback
```json
{
  "T": 1001,
  "pan": <pan_angle>,
  "tilt": <tilt_angle>,
  "r": <roll>,
  "p": <pitch>,
  "y": <yaw>,
  "v": <voltage>
}
```

##### CMD_BASE_FEEDBACK_FLOW (T=131)

**Description:** Enable/disable continuous feedback flow

**Command:**
```json
{"T":131,"cmd":0}  // Disable
{"T":131,"cmd":1}  // Enable
```

**Function:** `setBaseInfoFeedbackMode()` in `ugv_advance.h`

**Effect:** When enabled (`cmd=1`), sends base info periodically via `baseInfoFeedback()`

**Default:** Enabled (baseFeedbackFlow = 1)

**Response:** None

##### CMD_GET_IMU_DATA (T=126)

**Description:** Get current IMU sensor data

**Command:**
```json
{"T":126}
```

**Response:**
```json
{
  "T":1002,
  "r":<roll>,
  "p":<pitch>,
  "y":<yaw>,
  "ax":<accel_x>,
  "ay":<accel_y>,
  "az":<accel_z>,
  "gx":<gyro_x>,
  "gy":<gyro_y>,
  "gz":<gyro_z>,
  "mx":<mag_x>,
  "my":<mag_y>,
  "mz":<mag_z>,
  "temp":<temperature>
}
```

**Function:** `getIMUData()` in `IMU_ctrl.h`

### WiFi Configuration Commands

##### CMD_WIFI_INFO (T=405)

**Description:** Get current WiFi status

**Command:**
```json
{"T":405}
```

**Function:** `wifiStatusFeedback()` in `wifi_ctrl.h`

**Response:** JSON with WiFi mode, SSID, IP address, etc.
```json
{
  "T": 405,
  "mode": <0-3>,
  "ap_ssid": "...",
  "ap_password": "...",
  "sta_ssid": "...",
  "sta_password": "...",
  "ip": "<IP_address>"
}
```

##### CMD_SET_AP (T=402)

**Description:** Configure AP mode

**Function:** `wifiModeAP()` in `wifi_ctrl.h`

**Command:**
```json
{"T":402,"ssid":"PT","password":"12345678"}
```

**Response:** WiFi status JSON

##### CMD_SET_STA (T=403)

**Description:** Configure STA mode (connect to WiFi)

**Function:** `wifiModeSTA()` in `wifi_ctrl.h`

**Command:**
```json
{"T":403,"ssid":"YourWiFi","password":"password"}
```

**Response:** WiFi status JSON

##### CMD_WIFI_APSTA (T=404)

**Description:** Configure AP+STA mode

**Command:**
```json
{
  "T":404,
  "ap_ssid":"PT",
  "ap_password":"12345678",
  "sta_ssid":"YourWiFi",
**Function:** `wifiModeAPSTA()` in `wifi_ctrl.h`

**Command:**
```json
{
  "T":404,
  "ap_ssid":"PT",
  "ap_password":"12345678",
  "sta_ssid":"YourWiFi",
  "sta_password":"password"
}
```

**Response:** WiFi status JSON

### Servo Configuration Commands

##### CMD_SET_SERVO_ID (T=501)

**Description:** Change servo ID

**Command:**
```json
{"T":501,"raw":1,"new":2}
```

**Parameters:**
- `raw`: Current servo ID
- `new`: New servo ID

**Function:** `changeID()` in `RoArm-M2_module.h`

**Usage:** Calibrate servo IDs (pan should be ID 2, tilt ID 1)

**Process:**
1. Unlocks servo EEPROM
2. Writes new ID
3. Locks EEPROM
4. Servo must be powered and responsive

**Response:** Success/failure message via Serial (if InfoPrint enabled)

##### CMD_SET_MIDDLE (T=502)

**Description:** Set current position as servo center

**Command:**
```json
{"T":502,"id":2}  // Set pan servo center
{"T":502,"id":1}  // Set tilt servo center
```

**Function:** `setMiddlePos()` in `RoArm-M2_module.h`

**Usage:** Calibration - physically center servo, then set middle position

**Process:**
1. Manually center servo (disable torque first: `{"T":210,"cmd":0}`)
2. Send command: `{"T":502,"id":2}` for pan, `{"T":502,"id":1}` for tilt
3. Servo stores current position as center (2047)
4. Re-enable torque: `{"T":210,"cmd":1}`

**Response:** Success/failure message via Serial (if InfoPrint enabled)

---

## Code Quality Analysis

### Strengths

1. **Modular Architecture**
   - Clear separation of concerns
   - Each module in separate header file
   - Easy to locate functionality

2. **Multiple Control Interfaces**
   - USB Serial, WiFi Web, ESP-NOW
   - Unified JSON command protocol
   - Flexible communication options

3. **Comprehensive Feedback**
   - Servo telemetry (position, speed, load, voltage, current, temp)
   - IMU data (roll, pitch, yaw, raw sensors)
   - Battery monitoring
   - System status

4. **Configuration Persistence**
   - LittleFS for flash storage
   - WiFi config survives reboot
   - Mission system for automation

5. **Safety Features**
   - Emergency stop function
   - Torque control for manual positioning
   - Position constraints (-180 to 180 for pan, -30 to 90 for tilt)

### Areas for Improvement

#### 1. Error Handling

**Current State:**
- Servo communication failures set `status = false` in feedback structure
- No retry mechanism for failed servo reads
- IMU initialization doesn't check for sensor presence
- WiFi connection failures not well handled
- JSON parse errors are silent (no error response)
- Missing parameters may cause undefined behavior

**Specific Issues:**

**Servo Communication:**
```cpp
// In getGimbalFeedback()
if(st.FeedBack(GIMBAL_PAN_ID)!=-1) {
    gimbalFeedback[0].status = true;
    // ... read data
} else {
    gimbalFeedback[0].status = false;  // No retry
}
```
- Single attempt, no retry
- No timeout handling
- Status persists until next successful read

**Servo Initialization Check:**
```cpp
// In setup()
RoArmM2_initCheck(false);  // Checks BASE, SHOULDER, ELBOW servos
// But for gimbal, only PAN and TILT are needed
```
- Checks wrong servos for gimbal mode
- Should check GIMBAL_PAN_ID and GIMBAL_TILT_ID instead

**JSON Parsing:**
```cpp
// In serialCtrl()
DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
if (err == DeserializationError::Ok) {
    jsonCmdReceiveHandler();
} else {
    // Silent failure - no error response
}
```

**Recommendations:**
- Add retry logic for servo communication (3 attempts with delays)
- Check IMU sensor presence on init (verify I2C communication)
- Implement connection timeout handling for WiFi
- Add error counters and recovery procedures
- Send error responses for JSON parse failures
- Add servo-specific initialization check for gimbal mode
- Implement watchdog for stuck operations

#### 2. Safety Limits

**Current State:**
- Position constraints in `gimbalCtrlSimple()` and `gimbalCtrlMove()`
  - Pan: -180° to 180° (enforced)
  - Tilt: -30° to 90° (enforced)
- Speed constraints in `gimbalCtrlMove()` (1-2500, enforced)
- No acceleration limits (can be set to any value)
- No velocity limits during movement (servo can move at max speed)
- No load/temperature monitoring for safety shutdown
- No position limits checking in feedback loop (only on command input)

**Current Safety Features:**
- Emergency stop function (`gimbalCtrlStop()`)
- Torque control for manual positioning
- Heartbeat timeout (stops motors if no commands)
- Position constraints on input

**Missing Safety Features:**
- No maximum acceleration limits
- No velocity limits during movement
- No emergency stop on excessive servo load
- No emergency stop on high temperature
- No position limit checking in feedback (could drift)
- No soft limits (gradual slowdown near limits)

**Recommendations:**
- Add maximum acceleration limits (prevent sudden movements)
- Add velocity limits during movement (prevent overspeed)
- Add emergency stop on excessive load/temperature
  - Monitor `gimbalFeedback[].load` and `gimbalFeedback[].temper`
  - Shutdown if load > threshold or temp > 70°C
- Add position limits checking in feedback loop
  - Verify actual position stays within limits
  - Correct if servo drifts outside limits
- Add soft limits (gradual slowdown near limits)
- Add movement timeout (stop if movement takes too long)

#### 3. Performance

**Current State:**
- Main loop runs continuously (no delays)
- Servo feedback read every loop iteration
- IMU updated every loop iteration
- OLED updated every 10 seconds
- No loop timing control or monitoring

**Timing Analysis:**

**Loop Execution Time (estimated):**
- Serial read: ~1-5ms (if data available)
- HTTP handler: ~1-10ms (if request pending)
- Gimbal feedback: ~10-20ms (UART communication)
- IMU update: ~1-5ms (I2C read)
- Motor PID: ~1ms (if enabled)
- OLED update: ~50-100ms (when triggered, every 10s)

**Potential Issues:**
- No loop timing control (could vary significantly)
- Servo communication may block (UART read/write)
- No priority scheduling (all operations equal priority)
- IMU update every loop may be excessive (could be slower)
- No performance monitoring

**Servo Communication:**
```cpp
// Synchronous UART communication
st.FeedBack(GIMBAL_PAN_ID);  // Blocks until response or timeout
st.SyncWritePosEx(...);       // Blocks during transmission
```
- Blocks main loop during communication
- No timeout handling visible
- Could cause timing issues

**Recommendations:**
- Add FreeRTOS tasks for time-critical operations
  - Separate task for servo communication
  - Separate task for IMU reading
  - Main loop for command handling
- Implement non-blocking servo communication
  - Use queues for servo commands
  - Background task for servo updates
- Add loop timing monitoring
  - Measure loop execution time
  - Warn if loop takes too long
  - Optimize slow operations
- Optimize IMU update rate
  - Current: Every loop (~1000+ times/second)
  - Recommended: 100-200 Hz (5-10ms interval)
- Add performance metrics
  - Loop time statistics
  - Communication latency
  - Memory usage monitoring

#### 4. Code Organization

**Current State:**
- Many functions in header files (`.h` files contain implementations)
- Some large header files:
  - `RoArm-M2_module.h`: ~1300 lines (not used in PT mode)
  - `uart_ctrl.h`: ~520 lines
  - `gimbal_module.h`: ~225 lines
- Mixed concerns in some modules
- Some functions could be better organized

**File Organization Issues:**

**Header Files with Implementations:**
- `gimbal_module.h` - All functions implemented inline
- `uart_ctrl.h` - All handlers implemented inline
- `IMU_ctrl.h` - Functions implemented inline
- `oled_ctrl.h` - Functions implemented inline
- `battery_ctrl.h` - Functions implemented inline

**Benefits of Current Approach:**
- Faster compilation (no separate .cpp files)
- Easier to find implementations
- No linking issues

**Drawbacks:**
- Larger compilation units
- More recompilation on changes
- Harder to optimize
- No separation of interface/implementation

**Recommendations:**
- Move implementations to `.cpp` files for:
  - Large modules (gimbal_module, uart_ctrl)
  - Frequently changed code
  - Performance-critical functions
- Keep inline implementations for:
  - Small utility functions
  - Template functions
  - Simple getters/setters
- Split large modules:
  - `RoArm-M2_module.h` → Multiple files (base, shoulder, elbow, eoat)
  - `uart_ctrl.h` → Command handlers separate from parsing
- Separate concerns:
  - Control logic separate from communication
  - Hardware abstraction separate from application logic
- Add namespace organization:
  - `Gimbal::` namespace for gimbal functions
  - `Servo::` namespace for servo functions
  - `IMU::` namespace for IMU functions

#### 5. Documentation

**Current State:**
- Some inline comments (variable names are descriptive)
- Command definitions in `json_cmd.h` with examples (good!)
- Limited function documentation (no Doxygen-style comments)
- Algorithm details not documented
- No usage examples in code

**Existing Documentation:**
- JSON command examples in `json_cmd.h` (excellent!)
- Some comments explaining complex logic
- Variable names are generally descriptive

**Missing Documentation:**
- Function purpose and parameters
- Algorithm explanations
- Return values and error conditions
- Usage examples
- Hardware interface details
- Timing requirements

**Recommendations:**
- Add Doxygen-style function documentation:
  ```cpp
  /**
   * @brief Control gimbal to absolute position
   * @param Xinput Pan angle in degrees (-180 to 180)
   * @param Yinput Tilt angle in degrees (-30 to 90)
   * @param spdInput Speed (0=fastest)
   * @param accInput Acceleration (0=fastest)
   */
  void gimbalCtrlSimple(float Xinput, float Yinput, float spdInput, float accInput);
  ```
- Document algorithm details:
  - Position mapping formulas
  - IMU stabilization algorithm
  - User control state machine
- Add usage examples in comments
- Create API documentation (this document)
- Document hardware requirements
- Document timing constraints

#### 6. Calibration

**Current State:**
- Servo center position can be set (T=502)
- Servo ID can be changed (T=501)
- No IMU calibration routine (commented out in `IMU_ctrl.h`)
- No automated calibration sequence
- Calibration data not stored persistently

**Servo Calibration:**
- Manual process: Disable torque, center physically, set middle
- No verification that calibration succeeded
- No stored calibration offsets

**IMU Calibration:**
```cpp
// In IMU_ctrl.h - commented out
void imuCalibration() {
    // All code commented out
}
```
- No calibration implemented
- IMU may have offset errors
- No way to compensate for mounting orientation

**Recommendations:**
- Implement IMU calibration:
  - Multi-step calibration (as designed but commented)
  - Store offsets in flash
  - Apply offsets automatically
- Add calibration wizard/sequence:
  - Guided calibration process
  - Verification steps
  - Save/load calibration data
- Store calibration data in flash:
  - IMU offsets
  - Servo center positions
  - Mounting orientation
- Add calibration verification:
  - Test calibration accuracy
  - Warn if calibration seems wrong
  - Provide recalibration option

#### 7. State Management

**Current State:**
- Global variables for state (scattered across files)
- No state machine for gimbal modes
- Steady mode state not persisted (resets on reboot)
- Module type not persisted (resets to default)
- No state validation or consistency checks

**State Variables:**
```cpp
// Scattered across multiple files
bool steadyMode = false;           // gimbal_module.h
byte moduleType = 0;               // ugv_config.h
byte mainType = 1;                 // ugv_config.h
bool baseFeedbackFlow = 1;         // ugv_config.h
byte espNowMode = 3;               // ugv_config.h
bool RoArmM2_initCheckSucceed;     // ugv_config.h
```

**Issues:**
- State not persisted (resets on reboot)
- No state machine (direct variable access)
- No state validation
- No state transition logic
- State can become inconsistent

**Recommendations:**
- Implement state machine:
  - Define gimbal states (IDLE, MOVING, STEADY, ERROR)
  - State transition functions
  - State validation
- Persist important state:
  - Steady mode setting → `devConfig.json`
  - Module type → `devConfig.json`
  - WiFi settings → `wifiConfig.json` (already done)
- Add state validation:
  - Check state consistency on startup
  - Validate state transitions
  - Recover from invalid states
- Implement state transitions:
  - Controlled state changes
  - State change callbacks
  - State history/logging

### Code Metrics

**File Sizes:**
- Main file: ~260 lines
- Largest module: `RoArm-M2_module.h` (~1300 lines, not used in PT mode)
- `gimbal_module.h`: ~225 lines
- `json_cmd.h`: ~575 lines (mostly definitions)
- `uart_ctrl.h`: ~520 lines

**Complexity:**
- Main loop: Medium complexity
- Gimbal control: Low-Medium complexity
- JSON handler: High complexity (large switch statement)
- Communication: Medium complexity

**Dependencies:**
- 11 external libraries
- Multiple hardware interfaces
- File system operations
- Network stack

---

## System Diagrams

### Architecture Overview

```
┌─────────────────────────────────────────────────────────┐
│                    ESP32-D0WD-V3                        │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐ │
│  │   Main Loop  │  │  JSON Parser │  │  HTTP Server │ │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘ │
│         │                 │                  │         │
│  ┌──────▼─────────────────▼──────────────────▼───────┐ │
│  │         Command Handler (uart_ctrl.h)             │ │
│  └──────┬────────────────────────────────────────────┘ │
│         │                                              │
│  ┌──────▼──────┐  ┌──────────────┐  ┌──────────────┐ │
│  │   Gimbal    │  │     IMU      │  │    Servo     │ │
│  │   Control   │  │   Control    │  │  Interface   │ │
│  └─────────────┘  └──────────────┘  └──────────────┘ │
└─────────────────────────────────────────────────────────┘
         │                  │                  │
    ┌────▼────┐        ┌────▼────┐      ┌────▼────┐
    │  PAN    │        │ QMI8658  │      │ ST3215   │
    │ Servo   │        │ AK09918 │      │ Servos   │
    │ (ID 2)  │        │   IMU   │      │ (ID 1,2) │
    └─────────┘        └─────────┘      └──────────┘
    ┌────▼────┐
    │  TILT   │
    │ Servo   │
    │ (ID 1)  │
    └─────────┘
```

### Control Flow

```
Start
  │
  ├─► Serial Init (115200 baud)
  ├─► I2C Init (SDA:32, SCL:33)
  ├─► Battery Monitor Init (INA219 @ 0x42)
  ├─► Module Config (mainType=1, moduleType=2)
  ├─► OLED Init (SSD1306 @ 0x3C)
  ├─► IMU Init (QMI8658 + AK09918C)
  ├─► LED Init (GPIO 4,5 PWM)
  ├─► LittleFS Init (mount filesystem)
  ├─► Motion Pins Init (12V switch control)
  ├─► Servo Init (UART2: GPIO 18/19)
  ├─► Servo Status Check (PAN ID=2, TILT ID=1)
  ├─► Servo Move to Init Position
  ├─► WiFi Init (load config, start AP/STA)
  ├─► HTTP Server Init (port 80)
  ├─► ESP-NOW Init (mode 3 = follower)
  ├─► Encoder Init (for motor control)
  ├─► PID Init (for motor control)
  └─► Boot Mission Execute
       │
       ▼
    Main Loop (continuous, no delays)
       │
       ├─► serialCtrl() → Parse JSON → Handle Command
       ├─► server.handleClient() → HTTP requests
       ├─► moduleType_Gimbal()
       │   ├─► getGimbalFeedback() → Read servo data
       │   └─► gimbalSteady() → IMU compensation
       ├─► jsonCmdReceiveHandler() → Process queued commands
       ├─► Motor PID Control (Left/Right)
       ├─► oledInfoUpdate() → Update display (every 10s)
       ├─► updateIMUData() → Read IMU sensors
       ├─► baseInfoFeedback() → Send status (if enabled)
       └─► heartBeatCtrl() → Timeout safety
            │
            └─► Loop continues...
```

### Heartbeat Control

**Function:** `heartBeatCtrl()`

**Purpose:** Safety timeout - stops movement if no commands received

**Algorithm:**
```cpp
if (millis() - lastCmdRecvTime > HEART_BEAT_DELAY) {
    if (!heartbeatStopFlag) {
        heartbeatStopFlag = true;
        setGoalSpeed(0, 0);  // Stop motors
    }
}
```

**Default Timeout:** 3000ms (configurable via T=136)

**Reset:** Any command (T=1, T=11, T=13) resets `lastCmdRecvTime`

### Data Flow

```
Input Sources:
  USB Serial ──┐
  HTTP Web  ───┼──► JSON Parser ──► Command Handler ──► Gimbal Control
  ESP-NOW   ───┘      (uart_ctrl)    (jsonCmdReceive)    (gimbal_module)

Gimbal Control Flow:
  Position Command (X, Y, SPD, ACC)
       │
       ├─► Input Validation (constrain angles)
       ├─► Angle to Position Conversion
       │   ├─► Pan:  angle → servo position (0-4095)
       │   └─► Tilt: angle → servo position (inverted)
       ├─► Speed/Accel Mapping
       └─► SyncWritePosEx() → ST3215 Servos
            │
            ▼
  ST3215 Servos (PAN ID=2, TILT ID=1)
            │
            ▼
  Servo Feedback (getGimbalFeedback)
            │
            ├─► Position, Speed, Load
            ├─► Voltage, Current, Temperature
            └─► Status (communication success)

IMU Stabilization Flow:
  IMU Sensors (QMI8658 + AK09918C)
       │
       ├─► updateIMUData() → Read sensors
       ├─► Calculate Euler angles (roll, pitch, yaw)
       └─► gimbalSteady()
            │
            ├─► Read icm_pitch
            ├─► Calculate: targetY = steadyGoalY - icm_pitch
            └─► gimbalCtrlSimple(0, targetY, 0, 0)
                 │
                 └─► Compensate tilt axis

Feedback Flow:
  System State
       │
       ├─► Servo Feedback → gimbalFeedback[2]
       ├─► IMU Data → icm_roll, icm_pitch, icm_yaw
       ├─► Battery → loadVoltage_V
       └─► baseInfoFeedback()
            │
            └─► JSON Response
                 ├─► Pan/Tilt angles
                 ├─► IMU data
                 ├─► Battery voltage
                 └─► Servo status
```

### Communication Protocol Flow

```
Client                    ESP32                    Hardware
  │                         │                         │
  │── JSON Command ────────►│                         │
  │                         │── Parse JSON ──────────┤
  │                         │── Route by T ──────────┤
  │                         │── Execute Function ────►│── Servo Command
  │                         │                         │── IMU Read
  │                         │◄── Get Feedback ───────┤
  │◄── JSON Response ───────│                         │
  │                         │                         │
```

---

## Key Technical Details

### Servo Position Mapping

**Pan Servo (ID 2):**
- Position 0 = -180° (full left)
- Position 2047 = 0° (center)
- Position 4095 = +180° (full right)
- Formula: `angle = (pos - 2047) * 360 / 4095`

**Tilt Servo (ID 1):**
- Position 0 = +90° (up)
- Position 2047 = 0° (center)
- Position 4095 = -30° (down, limited)
- Formula: `angle = (2047 - pos) * 360 / 4095` (inverted)
- Constrained to -30° to +90° range

### IMU Stabilization Math

**Pitch Compensation:**
```
target_tilt_angle = desired_angle - imu_pitch
```

**Example:**
- Desired angle: 0° (level)
- Base tilts forward: +10° pitch
- Compensation: 0° - 10° = -10°
- Servo moves to -10° to keep camera level

**Limitations:**
- Only compensates pitch (forward/backward tilt)
- No roll compensation (side-to-side tilt)
- No yaw compensation (rotation)

### JSON Command Processing

**Flow:**
1. Receive JSON string (terminated by `\n`)
2. Parse with `deserializeJson()`
3. Extract command type `T`
4. Route to handler via switch statement
5. Execute function with parameters
6. Generate response JSON (if needed)
7. Send response (Serial/HTTP/ESP-NOW)

**Error Handling:**
- JSON parse errors: Silent failure (no response)
- Missing parameters: Function may use defaults or fail
- Invalid values: Constrained by functions

### File System Operations

**LittleFS Usage:**
- Configuration files: `wifiConfig.json`, `devConfig.json`
- Mission files: `*.mission`
- Text-based storage (JSON format)

**Operations:**
- Create, read, delete files
- Append, insert, replace lines
- Scan directory

**Limitations:**
- No subdirectories
- File names limited
- No file size limits enforced

---

## Troubleshooting Guide

### Servo Communication Issues

**Symptoms:**
- Servo doesn't respond
- `status = false` in feedback
- Error JSON: `{"T":1005,"id":X,"status":0}`

**Solutions:**
1. Check servo power (12V supply)
2. Verify servo IDs (should be 1 and 2)
3. Check UART connections (GPIO 18/19)
4. Try recalibrating servo center position
5. Check servo temperature (may overheat)

### IMU Stabilization Not Working

**Symptoms:**
- Steady mode enabled but camera still tilts
- No compensation visible

**Solutions:**
1. Verify IMU initialization succeeded
2. Check IMU data: `{"T":126}` - should show pitch values
3. Calibrate IMU if needed (command T=127, currently disabled)
4. Verify `steadyMode` is true: Check with feedback command
5. Check IMU mounting orientation

### WiFi Connection Issues

**Symptoms:**
- Can't connect to AP
- Can't connect to STA mode
- Web interface not accessible

**Solutions:**
1. Check WiFi mode: `{"T":405}` - Get current status
2. Reset WiFi: `{"T":408}` then reconfigure
3. Clear NVS if corrupted: `{"T":604}`
4. Check WiFi config file: `{"T":200}` - Scan files
5. Recreate WiFi config: `{"T":407}` with parameters

### Position Control Issues

**Symptoms:**
- Gimbal doesn't move to commanded position
- Moves to wrong position
- Jittery movement

**Solutions:**
1. Check servo feedback: Verify position reading
2. Recalibrate center: `{"T":502,"id":1}` and `{"T":502,"id":2}`
3. Check position constraints: X should be -180 to 180, Y should be -30 to 90
4. Verify servo IDs: `{"T":501}` to check/change
5. Check for mechanical binding

### Performance Issues

**Symptoms:**
- Slow response to commands
- Laggy movement
- Delayed feedback

**Solutions:**
1. Check loop timing: Add timing measurements
2. Reduce servo update rate if needed
3. Disable unnecessary features (ESP-NOW, feedback flow)
4. Check for blocking operations
5. Monitor free heap: `esp_get_free_heap_size()`

---

## Summary

The pan-tilt firmware is a well-structured ESP32 application with:

- **Modular design** with clear separation of concerns
- **Multiple communication interfaces** (USB, WiFi, ESP-NOW)
- **Comprehensive control** with position, speed, acceleration, and IMU stabilization
- **Rich feedback** from servos, IMU, and power monitoring
- **Persistent configuration** via LittleFS
- **Mission system** for automation

**Key Features:**
- 2-axis gimbal control (pan ±180°, tilt -30° to +90°)
- IMU-based pitch stabilization
- Multiple control modes (absolute, continuous, user, shell)
- Web interface for remote control
- ESP-NOW for wireless multi-device control
- Comprehensive servo telemetry

**Areas for Enhancement:**
- Error handling and recovery
- Safety limits and monitoring
- Performance optimization
- Code organization (move implementations to .cpp)
- Documentation improvements
- IMU calibration implementation

The system is functional and well-designed for its purpose, with room for improvements in robustness and maintainability.

---

## Function Reference

### Core Functions

#### Gimbal Control Functions

**File:** `gimbal_module.h`

**`gimbalCtrlSimple(float Xinput, float Yinput, float spdInput, float accInput)`**
- Absolute position control with synchronized movement
- Parameters: Pan angle (-180 to 180), Tilt angle (-30 to 90), Speed, Acceleration
- Returns: void
- Side effects: Sends command to both servos simultaneously

**`gimbalCtrlMove(float Xinput, float Yinput, float spdInputX, float spdInputY)`**
- Continuous movement with independent axis speeds
- Parameters: Target angles, separate speeds for X and Y
- Returns: void
- Side effects: Moves servos at different speeds

**`gimbalCtrlStop()`**
- Emergency stop function
- Parameters: None
- Returns: void
- Side effects: Disables and re-enables servo torque

**`getGimbalFeedback()`**
- Reads feedback from both servos
- Parameters: None
- Returns: void
- Side effects: Updates `gimbalFeedback[2]` array
- Error handling: Sets `status = false` if communication fails

**`gimbalSteady(float inputBiasY)`**
- IMU-based stabilization
- Parameters: Target Y angle bias
- Returns: void
- Side effects: Adjusts tilt axis to compensate for pitch
- Condition: Only runs if `steadyMode == true`

**`gimbalSteadySet(bool inputCmd, float inputY)`**
- Enable/disable steady mode
- Parameters: Enable flag, target Y angle
- Returns: void
- Side effects: Sets `steadyMode` and `steadyGoalY`

**`gimbalUserCtrl(int inputX, int inputY, int inputSpd)`**
- UI control mode with incremental movement
- Parameters: X direction (-1/0/1/2), Y direction (-1/0/1/2), Speed
- Returns: void
- Side effects: Maintains goal positions, reads actual position when stopped

**`gimbalUserCtrlShell(float lSpd, float rSpd)`**
- Convert motor speeds to gimbal control
- Parameters: Left speed, Right speed
- Returns: void
- Side effects: Calls `gimbalUserCtrl()` with converted directions

#### Communication Functions

**File:** `uart_ctrl.h`

**`serialCtrl()`**
- Parse JSON commands from USB serial
- Parameters: None (reads from Serial)
- Returns: void
- Side effects: Parses JSON, calls `jsonCmdReceiveHandler()`

**`jsonCmdReceiveHandler()`**
- Route JSON commands to appropriate handlers
- Parameters: None (uses global `jsonCmdReceive`)
- Returns: void
- Side effects: Executes command functions based on `T` field

#### IMU Functions

**File:** `IMU_ctrl.h`

**`imu_init()`**
- Initialize IMU sensors
- Parameters: None
- Returns: void
- Side effects: Initializes QMI8658 and AK09918C

**`updateIMUData()`**
- Read and process IMU data
- Parameters: None
- Returns: void
- Side effects: Updates global IMU variables (icm_roll, icm_pitch, etc.)

**`getIMUData()`**
- Get current IMU data as JSON
- Parameters: None
- Returns: void
- Side effects: Sends JSON response via Serial

#### Servo Functions

**File:** `RoArm-M2_module.h`

**`getFeedback(byte servoID, bool returnType)`**
- Read servo feedback data
- Parameters: Servo ID, return type (false=print all, true=only on fail)
- Returns: bool (success)
- Side effects: Updates `servoFeedback[]` array

**`servoTorqueCtrl(byte servoID, u8 enableCMD)`**
- Enable/disable servo torque
- Parameters: Servo ID (254=broadcast), Enable (0=off, 1=on)
- Returns: void
- Side effects: Changes servo torque state

**`setMiddlePos(byte InputID)`**
- Set current position as servo center
- Parameters: Servo ID
- Returns: void
- Side effects: Writes to servo EEPROM

**`changeID(byte oldID, byte newID)`**
- Change servo ID
- Parameters: Old ID, New ID
- Returns: void
- Side effects: Writes new ID to servo EEPROM

#### System Functions

**File:** `pan_tilt_base_v0.9.ino`

**`setup()`**
- Main initialization function
- Parameters: None
- Returns: void
- Side effects: Initializes all systems

**`loop()`**
- Main program loop
- Parameters: None
- Returns: void (never returns)
- Side effects: Continuous operation

**`moduleType_Gimbal()`**
- Gimbal-specific control loop
- Parameters: None
- Returns: void
- Side effects: Updates gimbal feedback and stabilization

---

## Hardware Pin Assignments

### GPIO Pin Map

| GPIO | Function | Direction | Notes |
|------|----------|-----------|-------|
| 4 | IO4 / LED / Motor | Output | PWM channel 7, 200Hz |
| 5 | IO5 / LED / Motor | Output | PWM channel 8, 200Hz |
| 18 | Servo UART RX | Input | S_RXD, 1000000 baud |
| 19 | Servo UART TX | Output | S_TXD, 1000000 baud |
| 32 | I2C SDA | I/O | S_SDA, shared bus |
| 33 | I2C SCL | Output | S_SCL, shared bus |
| 21 | Motor A IN1 | Output | TB6612FNG control |
| 22 | Motor B IN1 | Output | TB6612FNG control |
| 23 | Motor B IN2 | Output | TB6612FNG control |
| 25 | Motor A PWM | Output | TB6612FNG control |
| 26 | Motor B PWM | Output | TB6612FNG control |
| 27 | Encoder B A | Input | Motor encoder |
| 34 | Encoder A B | Input | Motor encoder |
| 35 | Encoder A A | Input | Motor encoder |

### I2C Bus Devices

| Address | Device | Function |
|---------|--------|----------|
| 0x3C | SSD1306 OLED | Display (128x32) |
| 0x42 | INA219 | Power monitoring |
| Variable | QMI8658 | 6-axis IMU (accel + gyro) |
| Variable | AK09918C | 3-axis magnetometer |

---

## Memory Usage

**From build output:**
- **RAM:** 14.9% (48,876 bytes used of 327,680 bytes)
- **Flash:** 78.3% (1,026,625 bytes used of 1,310,720 bytes)

**Memory Management:**
- Static JSON documents (256-512 bytes each)
- Global state variables
- LittleFS for file storage
- WiFi and HTTP server buffers
- ESP-NOW message buffers

**Potential Issues:**
- No dynamic memory allocation visible
- Static buffers may limit scalability
- File system uses flash memory

---

## Timing Characteristics

### Update Rates

| Function | Rate | Notes |
|----------|------|-------|
| Main loop | Continuous | No fixed rate, depends on operations |
| Servo feedback | Every loop | ~10-20ms per servo (UART communication) |
| IMU update | Every loop | ~1-5ms (I2C communication) |
| OLED update | 10 seconds | Battery voltage display |
| Battery monitor | 10 seconds | INA219 reading |
| Base feedback | Configurable | Default: enabled, extra delay: 100ms |
| Heartbeat check | Every loop | 3 second timeout default |

### Communication Timing

| Interface | Typical Latency | Notes |
|-----------|----------------|-------|
| USB Serial | <1ms | Direct UART |
| HTTP Web | 10-50ms | Network dependent |
| ESP-NOW | 5-20ms | Wireless, peer-dependent |
| Servo UART | 10-20ms | Per servo, synchronous |

---

## Configuration Files

### wifiConfig.json

**Location:** LittleFS `/wifiConfig.json`

**Format:**
```json
{
  "wifi_mode_on_boot": 3,
  "sta_ssid": "YourWiFi",
  "sta_password": "password",
  "ap_ssid": "PT",
  "ap_password": "12345678"
}
```

**Management:**
- Created via commands T=406 (from status) or T=407 (from input)
- Loaded automatically on boot
- Persists across reboots

### devConfig.json

**Location:** LittleFS `/devConfig.json`

**Purpose:** Device-specific configuration

**Management:** Via file operations (T=200-208)

### Mission Files

**Location:** LittleFS `/*.mission`

**Format:**
- First line: Mission metadata (JSON)
- Subsequent lines: JSON commands (one per line)

**Example:**
```
{"name":"boot","intro":"these cmds run automatically at boot."}
{"T":2,"P":20,"I":2500,"D":0,"L":255}
{"T":133,"X":0,"Y":0,"SPD":0,"ACC":0}
{"T":111,"cmd":1000}
```

**Management:** Via mission commands (T=220-242)

---

## Testing and Validation

### Recommended Tests

1. **Servo Communication Test**
   - Send position commands
   - Verify feedback matches commanded position
   - Test error handling (disconnect servo)

2. **IMU Stabilization Test**
   - Enable steady mode
   - Tilt base physically
   - Verify camera stays level

3. **Position Limits Test**
   - Command positions at limits (-180, 180, -30, 90)
   - Verify servos don't exceed limits
   - Test boundary conditions

4. **Communication Test**
   - Test all three interfaces (USB, WiFi, ESP-NOW)
   - Verify command routing
   - Test error responses

5. **Performance Test**
   - Measure loop timing
   - Test under load
   - Monitor memory usage

6. **Calibration Test**
   - Test servo center calibration
   - Verify calibration persistence
   - Test IMU calibration (when implemented)

---

## Conclusion

The pan-tilt firmware is a **well-architected, feature-rich system** that successfully implements:

✅ **Multi-interface control** (USB, WiFi, ESP-NOW)  
✅ **IMU-based stabilization**  
✅ **Comprehensive feedback**  
✅ **Persistent configuration**  
✅ **Mission automation**

**Key Strengths:**
- Clean modular design
- Extensive command set
- Good hardware abstraction
- Multiple control modes

**Areas for Enhancement:**
- Error handling and recovery
- Safety monitoring
- Performance optimization
- Code organization
- Documentation
- Calibration systems

The codebase demonstrates solid engineering practices and is production-ready with the noted improvements for enhanced robustness and maintainability.

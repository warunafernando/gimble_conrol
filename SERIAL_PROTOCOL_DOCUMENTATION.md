# ESP32 Pan-Tilt Serial Bus Communication Protocol

**Firmware Version:** 0.9  
**Date:** January 26, 2026

---

## Table of Contents

1. [Protocol Overview](#protocol-overview)
2. [Message Structure](#message-structure)
3. [Communication Flow](#communication-flow)
4. [Command Categories](#command-categories)
5. [Feedback Message Types](#feedback-message-types)
6. [Implementation Details](#implementation-details)
7. [Protocol Characteristics](#protocol-characteristics)
8. [Example Communication Sequences](#example-communication-sequences)
9. [Complete Command Reference](#complete-command-reference)

---

## Protocol Overview

The ESP32 pan_tilt_base_v0.9 firmware uses a **JSON-based serial communication protocol** over USB Serial (UART). All communication is text-based JSON messages terminated by newline characters.

### Communication Parameters

- **Baud Rate**: 115200
- **Serial Port**: USB Serial (CP2102 UART bridge)
- **Message Format**: JSON objects
- **Message Termination**: Newline character (`\n`)
- **Library**: ArduinoJson (StaticJsonDocument)
  - `jsonCmdReceive`: 256 bytes (incoming commands)
  - `jsonInfoSend`: 256 bytes (outgoing responses)
  - `jsonInfoHttp`: 512 bytes (HTTP/feedback responses)

### Protocol Type

- **Text-based**: Human-readable JSON format
- **Line-delimited**: Messages terminated by `\n`
- **Stateless**: Each command is independent
- **Asynchronous**: ESP32 can send feedback at any time
- **Bidirectional**: Commands and responses on same serial port
- **No framing**: Relies on JSON parsing and newline detection

---

## Message Structure

### Command Format (Host → ESP32)

All commands are JSON objects with at minimum a `"T"` field indicating the command type:

```json
{"T":<command_type>,"param1":value1,"param2":value2}
```

**Required Field:**
- `"T"`: Command type (integer)

**Optional Fields:**
- Command-specific parameters (varies by command type)
- Parameter types: integer, float, string, boolean

**Example Commands:**
```json
{"T":126}
{"T":133,"X":45,"Y":10,"SPD":0,"ACC":0}
{"T":137,"s":1,"y":0}
```

### Response Format (ESP32 → Host)

All responses are JSON objects with a `"T"` field indicating the response type:

```json
{"T":<response_type>,"field1":value1,"field2":value2}
```

**Response Types:**
- `T=1001`: Base info feedback (periodic)
- `T=1002`: IMU data feedback
- `T=1003`: ESP-NOW receive notification
- `T=1004`: ESP-NOW send status
- `T=1005`: Servo error feedback

**Example Responses:**
```json
{"T":1002,"r":2.5,"p":-1.2,"y":45.8,"ax":0.1,"ay":0.0,"az":9.8,"gx":0.001,"gy":0.000,"gz":0.000,"mx":1234,"my":567,"mz":890,"temp":25.5}
{"T":1001,"r":2.5,"p":-1.2,"y":45.8,"v":12.5,"pan":45.0,"tilt":10.0}
{"T":1005,"id":1,"status":0}
```

---

## Communication Flow

### Command Reception (Host → ESP32)

The ESP32 receives commands through the `serialCtrl()` function, which is called every loop iteration:

**Flow:**
1. **Serial Reading**: `serialCtrl()` checks `Serial.available()`
2. **Character Accumulation**: Characters read one-by-one and appended to `receivedData` string
3. **Message Detection**: When `\n` (newline) character is received, message is complete
4. **JSON Parsing**: `deserializeJson(jsonCmdReceive, receivedData)` parses JSON string
5. **Echo Mode**: If `uartCmdEcho` is enabled and `InfoPrint == 1`, command is echoed back
6. **Command Handler**: `jsonCmdReceiveHandler()` processes command based on `T` value
7. **Buffer Reset**: `receivedData` string cleared for next message

**Code Location:** `uart_ctrl.h` - `serialCtrl()` function

**Error Handling:**
- JSON parse errors are silently ignored (no response sent)
- Invalid command types are ignored
- Missing required parameters may cause command to be skipped

### Response Transmission (ESP32 → Host)

The ESP32 sends responses through `Serial.println()`:

**Flow:**
1. **JSON Construction**: Response data built in `jsonInfoHttp` or `jsonInfoSend` StaticJsonDocument
2. **Field Assignment**: Response fields populated based on command type
3. **Serialization**: `serializeJson(jsonInfoHttp, getInfoJsonString)` converts JSON object to string
4. **Transmission**: `Serial.println(getInfoJsonString)` sends JSON string with newline

**Code Locations:**
- `ugv_advance.h` - `baseInfoFeedback()` (T=1001)
- `IMU_ctrl.h` - `getIMUData()` (T=1002)
- `gimbal_module.h` - Error feedback (T=1005)

---

## Command Categories

### 1. UGV/Motion Control (T: 1-13)

**T=1: Speed Control**
```json
{"T":1,"L":0.5,"R":0.5}
```
- `L`: Left motor speed (-1.0 to 1.0)
- `R`: Right motor speed (-1.0 to 1.0)
- Resets heartbeat timer

**T=11: PWM Input**
```json
{"T":11,"L":164,"R":164}
```
- `L`: Left motor PWM (-255 to 255)
- `R`: Right motor PWM (-255 to 255)
- Disables PID compute mode
- Resets heartbeat timer

**T=13: ROS Control**
```json
{"T":13,"X":0.1,"Z":0.3}
```
- `X`: Linear velocity (m/s)
- `Z`: Angular velocity (rad/s)
- Requires encoders
- Resets heartbeat timer

**T=2: Motor PID Settings**
```json
{"T":2,"P":200,"I":2500,"D":0,"L":255}
```
- `P`: Proportional gain
- `I`: Integral gain
- `D`: Derivative gain
- `L`: Windup limit

### 2. Gimbal Control (T: 133-141)

**T=133: Simple Gimbal Control**
```json
{"T":133,"X":45,"Y":10,"SPD":0,"ACC":0}
```
- `X`: Pan angle (-180° to +180°)
- `Y`: Tilt angle (-30° to +90°)
- `SPD`: Speed (0=fastest)
- `ACC`: Acceleration (0=fastest)
- Moves to absolute position

**T=134: Gimbal Move Control**
```json
{"T":134,"X":45,"Y":10,"SX":300,"SY":300}
```
- `X`: Target pan angle
- `Y`: Target tilt angle
- `SX`: Pan speed (steps/s)
- `SY`: Tilt speed (steps/s)
- Continuous movement

**T=135: Gimbal Stop**
```json
{"T":135}
```
- Emergency stop
- Disables torque for 3ms, then re-enables

**T=137: Gimbal Steady Mode**
```json
{"T":137,"s":1,"y":0}
```
- `s`: Enable (1) or disable (0) steady mode
- `y`: Target tilt angle for steady mode
- Enables IMU-based pitch stabilization

**T=141: Gimbal User Control**
```json
{"T":141,"X":0,"Y":0,"SPD":300}
```
- `X`: Pan direction (-1=decrease, 0=stop, 1=increase, 2=middle)
- `Y`: Tilt direction (-1=decrease, 0=stop, 1=increase, 2=middle)
- `SPD`: Movement speed
- Relative control

### 3. IMU Commands (T: 126-129)

**T=126: Get IMU Data**
```json
{"T":126}
```
- **Response**: `{"T":1002,"r":...,"p":...,"y":...,"ax":...,"ay":...,"az":...,"gx":...,"gy":...,"gz":...,"mx":...,"my":...,"mz":...,"temp":...}`
- Returns current IMU sensor data

**T=127: IMU Calibration**
```json
{"T":127}
```
- Calibrates IMU (currently disabled/commented out)
- Takes ~5 seconds
- Device must be kept still

**T=128: Get IMU Offset**
```json
{"T":128}
```
- Returns magnetometer offset values (currently disabled)

**T=129: Set IMU Offset**
```json
{"T":129,"x":-12,"y":0,"z":0}
```
- `x`: X-axis offset
- `y`: Y-axis offset
- `z`: Z-axis offset
- Sets magnetometer calibration offsets

### 4. Feedback Control (T: 130-131, 142-143)

**T=130: Get Base Feedback**
```json
{"T":130}
```
- **Response**: `{"T":1001,...}`
- One-time request for base information
- Returns IMU angles, voltage, pan/tilt or arm positions

**T=131: Base Feedback Flow Control**
```json
{"T":131,"cmd":1}
```
- `cmd`: 0=disable, 1=enable periodic feedback
- Default: enabled (1)
- Controls automatic periodic status updates

**T=142: Feedback Flow Interval**
```json
{"T":142,"cmd":100}
```
- `cmd`: Delay in milliseconds between feedback messages
- Default: 100ms
- Controls feedback update rate

**T=143: UART Echo Mode**
```json
{"T":143,"cmd":1}
```
- `cmd`: 0=off, 1=on
- Default: off (0)
- Echoes received commands back to serial

### 5. Robot Arm Control (T: 100-123)

**T=100: Move to Init Position**
```json
{"T":100}
```
- Moves all joints to initialization position
- No interpolation

**T=101: Single Joint Control (Radians)**
```json
{"T":101,"joint":1,"rad":0,"spd":0,"acc":10}
```
- `joint`: 1=BASE, 2=SHOULDER, 3=ELBOW, 4=EOAT
- `rad`: Target angle in radians
- `spd`: Speed (steps/s)
- `acc`: Acceleration (steps/s²)

**T=102: All Joints Control (Radians)**
```json
{"T":102,"base":0,"shoulder":0,"elbow":1.57,"hand":1.57,"spd":0,"acc":10}
```
- `base`: Base joint angle (radians)
- `shoulder`: Shoulder joint angle (radians)
- `elbow`: Elbow joint angle (radians)
- `hand`: Hand/EOAT joint angle (radians)
- `spd`: Speed
- `acc`: Acceleration

**T=103: Single Axis Control**
```json
{"T":103,"axis":2,"pos":0,"spd":0.25}
```
- `axis`: 1=X, 2=Y, 3=Z, 4=T (theta)
- `pos`: Target position
- `spd`: Speed

**T=104: XYZT Goal Control**
```json
{"T":104,"x":235,"y":0,"z":234,"t":3.14,"spd":0.25}
```
- `x`: X position (mm)
- `y`: Y position (mm)
- `z`: Z position (mm)
- `t`: Theta angle (radians)
- `spd`: Speed
- Uses Bessel interpolation

**T=105: Servo Feedback**
```json
{"T":105}
```
- Returns current arm position and torque data
- Includes: x, y, z, t, torB, torS, torE

**T=121: Single Joint Angle (Degrees)**
```json
{"T":121,"joint":1,"angle":0,"spd":10,"acc":10}
```
- `joint`: 1-4 (BASE, SHOULDER, ELBOW, EOAT)
- `angle`: Target angle in degrees
- `spd`: Speed (angle/s)
- `acc`: Acceleration (angle/s², max: 22.5)

**T=122: All Joints Angle Control**
```json
{"T":122,"b":0,"s":0,"e":90,"h":180,"spd":10,"acc":10}
```
- `b`: Base angle (degrees)
- `s`: Shoulder angle (degrees)
- `e`: Elbow angle (degrees)
- `h`: Hand angle (degrees)
- `spd`: Speed (angle/s)
- `acc`: Acceleration (angle/s²)

**T=123: Constant Control**
```json
{"T":123,"m":0,"axis":0,"cmd":0,"spd":3}
```
- `m`: Mode (0=angle, 1=xyzt)
- `axis`: Axis number
- `cmd`: 0=stop, 1=increase, 2=decrease
- `spd`: Speed

### 6. File System Commands (T: 200-208)

**T=200: Scan Files**
```json
{"T":200}
```
- Lists all files in LittleFS
- Prints file names and first lines

**T=201: Create File**
```json
{"T":201,"name":"file.txt","content":"inputContentHere."}
```
- Creates new file with content
- Overwrites if exists

**T=202: Read File**
```json
{"T":202,"name":"file.txt"}
```
- Reads and prints entire file content

**T=203: Delete File**
```json
{"T":203,"name":"file.txt"}
```
- Deletes specified file

**T=204: Append Line**
```json
{"T":204,"name":"file.txt","content":"new line"}
```
- Appends line to end of file

**T=205: Insert Line**
```json
{"T":205,"name":"file.txt","lineNum":3,"content":"content"}
```
- Inserts line at specified line number

**T=206: Replace Line**
```json
{"T":206,"name":"file.txt","lineNum":3,"content":"Content"}
```
- Replaces line at specified line number

**T=207: Read Line**
```json
{"T":207,"name":"file.txt","lineNum":3}
```
- Reads single line from file

**T=208: Delete Line**
```json
{"T":208,"name":"file.txt","lineNum":3}
```
- Deletes single line from file

### 7. Mission Control (T: 220-242)

**T=220: Create Mission**
```json
{"T":220,"name":"mission_a","intro":"test mission"}
```
- Creates new mission file
- Mission files stored as `.mission` in LittleFS

**T=221: Get Mission Content**
```json
{"T":221,"name":"mission_a"}
```
- Returns all steps in mission

**T=222: Append Step (JSON)**
```json
{"T":222,"name":"mission_a","step":"{\"T\":104,\"x\":235,\"y\":0,\"z\":234,\"t\":3.14,\"spd\":0.25}"}
```
- Appends JSON command as step
- Step is escaped JSON string

**T=223: Append Step (Feedback)**
```json
{"T":223,"name":"mission_a","spd":0.25}
```
- Appends current state as step

**T=224: Append Delay**
```json
{"T":224,"name":"mission_a","delay":3000}
```
- Appends delay command (milliseconds)

**T=225-230: Insert/Replace Steps**
- Similar to append but at specific step number

**T=231: Delete Step**
```json
{"T":231,"name":"mission_a","stepNum":3}
```
- Deletes step at specified number

**T=241: Move to Step**
```json
{"T":241,"name":"mission_a","stepNum":3}
```
- Jumps to specific step in mission

**T=242: Play Mission**
```json
{"T":242,"name":"mission_a","times":3}
```
- `times`: Number of repeats (-1 = infinite loop)
- Executes mission steps sequentially

### 8. WiFi Control (T: 401-408)

**T=401: WiFi Mode on Boot**
```json
{"T":401,"cmd":3}
```
- `cmd`: 0=OFF, 1=AP, 2=STA, 3=AP+STA
- Sets WiFi mode for next boot
- Saved to `wifiConfig.json`

**T=402: Set AP Mode**
```json
{"T":402,"ssid":"UGV","password":"12345678"}
```
- Configures Access Point mode
- IP: 192.168.4.1

**T=403: Set STA Mode**
```json
{"T":403,"ssid":"network_name","password":"password"}
```
- Connects to WiFi network
- Timeout: 15 seconds

**T=404: Set AP+STA Mode**
```json
{"T":404,"ap_ssid":"UGV","ap_password":"12345678","sta_ssid":"network","sta_password":"pass"}
```
- Both AP and STA active simultaneously

**T=405: Get WiFi Info**
```json
{"T":405}
```
- Returns current WiFi status
- Includes: mode, SSID, IP addresses

**T=406: Create WiFi Config (from status)**
```json
{"T":406}
```
- Creates `wifiConfig.json` from current settings

**T=407: Create WiFi Config (from input)**
```json
{"T":407,"mode":3,"ap_ssid":"UGV","ap_password":"12345678","sta_ssid":"network","sta_password":"pass"}
```
- Creates `wifiConfig.json` from provided parameters

**T=408: Stop WiFi**
```json
{"T":408}
```
- Disconnects WiFi

### 9. Servo Settings (T: 501-503)

**T=501: Change Servo ID**
```json
{"T":501,"raw":1,"new":11}
```
- `raw`: Current servo ID
- `new`: New servo ID
- Changes servo identifier

**T=502: Set Middle Position**
```json
{"T":502,"id":11}
```
- Sets current position as middle/zero position
- Servo IDs: 11=BASE, 12=SHOULDER_DRIVING, 13=SHOULDER_DRIVEN, 14=ELBOW, 15=GRIPPER
- For gimbal: 1=TILT, 2=PAN

**T=503: Set Servo PID**
```json
{"T":503,"id":14,"p":16}
```
- `id`: Servo ID
- `p`: P gain value
- Configures servo PID parameters

### 10. ESP-NOW Commands (T: 300-306)

**T=300: Broadcast Follower Mode**
```json
{"T":300,"mode":1,"mac":"FF:FF:FF:FF:FF:FF"}
```
- `mode`: 0=whitelist only, 1=allow broadcast
- `mac`: Whitelist MAC address
- Controls broadcast command acceptance

**T=301: ESP-NOW Mode Config**
```json
{"T":301,"mode":3}
```
- `mode`: 0=None, 1=Leader Group, 2=Leader Single, 3=Follower
- Sets ESP-NOW operation mode

**T=302: Get MAC Address**
```json
{"T":302}
```
- Returns device MAC address

**T=303: Add Follower**
```json
{"T":303,"mac":"CC:DB:A7:5B:E4:1C"}
```
- Adds MAC address to peer list

**T=304: Remove Follower**
```json
{"T":304,"mac":"CC:DB:A7:5B:E4:1C"}
```
- Removes MAC address from peer list

**T=305: ESP-NOW Group Control**
```json
{"T":305,"dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
```
- Sends to multiple peers
- Broadcast MAC (FF:FF:FF:FF:FF:FF) not allowed

**T=306: ESP-NOW Single Control**
```json
{"T":306,"mac":"FF:FF:FF:FF:FF:FF","dev":0,"b":0,"s":0,"e":1.57,"h":1.57,"cmd":0,"megs":"hello!"}
```
- Sends to single device or broadcast
- Can include nested JSON in `megs` field

### 11. System Control (T: 600-605, 900)

**T=600: Reboot Device**
```json
{"T":600}
```
- Immediately reboots ESP32

**T=601: Free Flash Space**
```json
{"T":601}
```
- Returns available flash space

**T=602: Boot Mission Info**
```json
{"T":602}
```
- Returns boot mission content

**T=603: Reset Boot Mission**
```json
{"T":603}
```
- Resets boot mission to default

**T=604: Clear NVS**
```json
{"T":604}
```
- Erases NVS (Non-Volatile Storage)
- Used for WiFi troubleshooting

**T=605: Info Print Mode**
```json
{"T":605,"cmd":1}
```
- `cmd`: 0=off, 1=debug info, 2=flow feedback
- Controls serial debug output

**T=900: Main/Module Type Settings**
```json
{"T":900,"main":1,"module":2}
```
- `main`: 1=WAVE ROVER, 2=UGV02, 3=UGV01
- `module`: 0=None, 1=RoArm, 2=Gimbal
- Configures device type

### 12. Other Commands

**T=3: OLED Control**
```json
{"T":3,"lineNum":0,"Text":"putYourTextHere"}
```
- Sets OLED display line text

**T=-3: OLED Default**
```json
{"T":-3}
```
- Resets OLED to default display

**T=4: Module Type**
```json
{"T":4,"cmd":2}
```
- `cmd`: 0=None, 1=RoArm, 2=Gimbal
- Changes active module type

**T=132: LED Control**
```json
{"T":132,"IO4":255,"IO5":255}
```
- Controls LED brightness (0-255)

**T=136: Heartbeat Delay**
```json
{"T":136,"cmd":3000}
```
- Sets heartbeat timeout (milliseconds)
- Default: 3000ms

**T=138-140: Speed Rate Control**
- Get/set/save speed rate multipliers

**T=144: Arm Control UI**
```json
{"T":144,"E":100,"Z":0,"R":0}
```
- UI-friendly arm control

**T=210: Torque Control**
```json
{"T":210,"cmd":1}
```
- `cmd`: 0=disable, 1=enable
- Controls servo torque lock
- Uses broadcast ID 254 (all servos)

---

## Feedback Message Types

### T=1001: Base Info Feedback

Periodic feedback sent when `baseFeedbackFlow = true`. Update rate controlled by `feedbackFlowExtraDelay` (default: 100ms).

**Format:**
```json
{
  "T": 1001,
  "r": 2.5,        // Roll angle (degrees)
  "p": -1.2,       // Pitch angle (degrees)
  "y": 45.8,       // Yaw angle (degrees)
  "v": 12.5,       // Battery voltage (V)
  "pan": 0.0,      // Pan angle (degrees) - if moduleType=2
  "tilt": 0.0      // Tilt angle (degrees) - if moduleType=2
}
```

**For Robot Arm (moduleType=1):**
```json
{
  "T": 1001,
  "r": 2.5,
  "p": -1.2,
  "y": 45.8,
  "v": 12.5,
  "x": 235.0,      // X position (mm)
  "y": 0.0,         // Y position (mm)
  "z": 234.0,       // Z position (mm)
  "b": 0.0,         // Base joint angle (radians)
  "s": 0.0,         // Shoulder joint angle (radians)
  "e": 1.57,        // Elbow joint angle (radians)
  "t": 0.0,         // Hand/EOAT angle (radians)
  "torB": 100,      // Base joint torque
  "torS": 50,       // Shoulder joint torque
  "torE": 30,       // Elbow joint torque
  "torH": 20        // Hand joint torque
}
```

**Function:** `baseInfoFeedback()` in `ugv_advance.h`

### T=1002: IMU Data Feedback

Response to `T=126` command. Returns complete IMU sensor data.

**Format:**
```json
{
  "T": 1002,
  "r": 2.5,         // Roll angle (degrees)
  "p": -1.2,        // Pitch angle (degrees)
  "y": 45.8,        // Yaw angle (degrees)
  "ax": 0.100,      // Accelerometer X (m/s²)
  "ay": 0.000,      // Accelerometer Y (m/s²)
  "az": 9.800,      // Accelerometer Z (m/s²)
  "gx": 0.001,      // Gyroscope X (rad/s)
  "gy": 0.000,      // Gyroscope Y (rad/s)
  "gz": 0.000,      // Gyroscope Z (rad/s)
  "mx": 1234,       // Magnetometer X (uT)
  "my": 567,        // Magnetometer Y (uT)
  "mz": 890,        // Magnetometer Z (uT)
  "temp": 25.5      // Temperature (°C, if available)
}
```

**Function:** `getIMUData()` in `IMU_ctrl.h`

### T=1003: ESP-NOW Receive Notification

Sent when ESP-NOW message is received.

**Format:**
```json
{
  "T": 1003,
  "mac": "FF:FF:FF:FF:FF:FF",
  "megs": "hello!"
}
```

### T=1004: ESP-NOW Send Status

Sent after ESP-NOW transmission attempt.

**Format:**
```json
{
  "T": 1004,
  "mac": "FF:FF:FF:FF:FF:FF",
  "status": 1,
  "megs": "xxx"
}
```

**Status Values:**
- 0: Failed
- 1: Succeed
- 2: Error initializing ESP-NOW
- 3: Invalid MAC address format
- 4: Failed to add peer
- 5: Add peer
- 6: Delete peer
- 7: Error sending the data
- 8: Sent with success

### T=1005: Servo Error Feedback

Sent when servo communication fails.

**Format:**
```json
{
  "T": 1005,
  "id": 1,
  "status": 0
}
```

- `id`: Servo ID (1=TILT, 2=PAN for gimbal)
- `status`: 0 = communication failure, 1 = OK

**Trigger:** When `st.FeedBack(servoID) == -1`

**Function:** Error reporting in `gimbal_module.h` and `RoArm-M2_module.h`

---

## Implementation Details

### Serial Reading Implementation

**File:** `uart_ctrl.h`

**Function:** `serialCtrl()`

```cpp
void serialCtrl() {
  static String receivedData;

  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    receivedData += receivedChar;

    if (receivedChar == '\n') {
      DeserializationError err = deserializeJson(jsonCmdReceive, receivedData);
      if (err == DeserializationError::Ok) {
        if (InfoPrint == 1 && uartCmdEcho) {
          Serial.print(receivedData);
        }
        jsonCmdReceiveHandler();
      }
      receivedData = "";
    }
  }
}
```

**Characteristics:**
- Called every loop iteration
- Uses static String buffer
- Accumulates characters until `\n`
- Silent error handling (parse errors ignored)
- Optional echo mode

### Command Processing Implementation

**File:** `uart_ctrl.h`

**Function:** `jsonCmdReceiveHandler()`

**Structure:**
- Switch statement on `jsonCmdReceive["T"]`
- Each case calls specific handler function
- Parameter extraction: `jsonCmdReceive["param"]`
- Type checking: `jsonCmdReceive.containsKey()`, `.is<float>()`

**Heartbeat Reset:**
- Motor control commands (T=1, T=11, T=13) reset `lastCmdRecvTime`
- Prevents timeout shutdown

**Error Handling:**
- Missing parameters: Command skipped
- Invalid types: Command skipped
- No error response sent to host

### Response Generation Implementation

**JSON Documents:**
- `jsonInfoHttp`: 512 bytes (for feedback/responses)
- `jsonInfoSend`: 256 bytes (for simple responses)
- `jsonCmdReceive`: 256 bytes (for incoming commands)

**Serialization:**
```cpp
String getInfoJsonString;
serializeJson(jsonInfoHttp, getInfoJsonString);
Serial.println(getInfoJsonString);
```

**Response Timing:**
- Immediate: Commands that request data (T=126, T=130)
- Periodic: Base feedback when `baseFeedbackFlow = true`
- Event-driven: Error feedback (T=1005) when servo fails

### Periodic Feedback Mechanism

**Function:** `baseInfoFeedback()` in `ugv_advance.h`

**Control:**
- `baseFeedbackFlow`: Enable/disable flag
- `feedbackFlowExtraDelay`: Delay between messages (default: 100ms)
- Called in main loop when `baseFeedbackFlow == true`

**Rate Limiting:**
```cpp
static unsigned long last_feedback_time;
if (millis() - last_feedback_time < feedbackFlowExtraDelay) {
    return;
}
```

---

## Protocol Characteristics

### 1. Text-based Protocol
- **Format**: Human-readable JSON
- **Advantages**: Easy to debug, no binary parsing needed
- **Disadvantages**: Larger message size, slower than binary

### 2. Line-delimited Messages
- **Termination**: Newline character (`\n`)
- **Parsing**: Relies on newline detection
- **Buffer**: String accumulates until `\n` received

### 3. Stateless Communication
- **No Session**: Each command is independent
- **No Sequence Numbers**: No message ordering guarantee
- **No Acknowledgments**: Commands don't require ACK (except data requests)

### 4. Asynchronous Operation
- **Bidirectional**: Commands and responses on same port
- **Non-blocking**: ESP32 can send feedback at any time
- **Interleaved**: Feedback may interrupt command processing

### 5. No Message Framing
- **Relies on**: JSON parsing and newline detection
- **Vulnerable to**: Incomplete messages, buffer overflows
- **Recovery**: Newline resets buffer

### 6. Error Handling
- **Parse Errors**: Silently ignored
- **Invalid Commands**: Skipped without response
- **Missing Parameters**: Command may be partially executed or skipped

---

## Example Communication Sequences

### Example 1: Get IMU Data

```
Host → ESP32: {"T":126}\n
ESP32 → Host: {"T":1002,"r":2.5,"p":-1.2,"y":45.8,"ax":0.1,"ay":0.0,"az":9.8,"gx":0.001,"gy":0.000,"gz":0.000,"mx":1234,"my":567,"mz":890,"temp":25.5}\n
```

### Example 2: Gimbal Control

```
Host → ESP32: {"T":133,"X":45,"Y":10,"SPD":0,"ACC":0}\n
ESP32 → Host: (no immediate response, gimbal moves to position)
```

### Example 3: Get Base Feedback

```
Host → ESP32: {"T":130}\n
ESP32 → Host: {"T":1001,"r":2.5,"p":-1.2,"y":45.8,"v":12.5,"pan":45.0,"tilt":10.0}\n
```

### Example 4: Enable Periodic Feedback

```
Host → ESP32: {"T":131,"cmd":1}\n
ESP32 → Host: (no response, but periodic feedback starts)
ESP32 → Host: {"T":1001,"r":2.5,"p":-1.2,"y":45.8,"v":12.5,"pan":45.0,"tilt":10.0}\n
ESP32 → Host: {"T":1001,"r":2.6,"p":-1.1,"y":45.9,"v":12.5,"pan":45.0,"tilt":10.0}\n
... (continues every 100ms)
```

### Example 5: Servo Error

```
ESP32 → Host: {"T":1005,"id":1,"status":0}\n
(Spontaneous error message when servo communication fails)
```

### Example 6: Gimbal Steady Mode

```
Host → ESP32: {"T":137,"s":1,"y":0}\n
ESP32 → Host: (no response, steady mode enabled)
(IMU stabilization now active for tilt axis)
```

### Example 7: Mission Playback

```
Host → ESP32: {"T":242,"name":"boot","times":1}\n
ESP32 → Host: (no response, mission executes)
(Each mission step executed sequentially)
```

---

## Complete Command Reference

### Quick Reference Table

| T Value | Command Name | Category | Response |
|---------|-------------|----------|----------|
| 1 | Speed Control | Motion | None |
| 2 | Motor PID | Motion | None |
| 3 | OLED Control | Display | None |
| 4 | Module Type | System | None |
| 11 | PWM Input | Motion | None |
| 13 | ROS Control | Motion | None |
| 126 | Get IMU Data | IMU | T=1002 |
| 127 | IMU Calibration | IMU | None |
| 130 | Get Base Feedback | Feedback | T=1001 |
| 131 | Feedback Flow | Feedback | None |
| 133 | Gimbal Simple | Gimbal | None |
| 134 | Gimbal Move | Gimbal | None |
| 135 | Gimbal Stop | Gimbal | None |
| 137 | Gimbal Steady | Gimbal | None |
| 200 | Scan Files | File | Text output |
| 210 | Torque Control | Servo | None |
| 220 | Create Mission | Mission | None |
| 242 | Play Mission | Mission | None |
| 300 | Broadcast Mode | ESP-NOW | None |
| 301 | ESP-NOW Config | ESP-NOW | None |
| 401 | WiFi Boot Mode | WiFi | None |
| 402 | Set AP | WiFi | None |
| 403 | Set STA | WiFi | None |
| 405 | WiFi Info | WiFi | JSON response |
| 501 | Change Servo ID | Servo | None |
| 502 | Set Middle | Servo | None |
| 600 | Reboot | System | None |

### Command Parameter Reference

**Common Parameters:**
- `T`: Command type (always required)
- `cmd`: Command value (0/1, enable/disable)
- `id`: Servo ID
- `X`, `Y`: Gimbal angles
- `L`, `R`: Left/Right motor values
- `P`, `I`, `D`: PID gains
- `name`: File or mission name
- `mac`: MAC address string

**Data Types:**
- Integers: Servo IDs, command types, counts
- Floats: Angles (degrees/radians), speeds, positions
- Strings: File names, MAC addresses, text content
- Booleans: Enable/disable flags (0/1 as integers)

---

## Files Referenced

### Core Implementation Files

- **`pan_tilt_base_v0.9.ino`**: Main program
  - Calls `serialCtrl()` every loop iteration
  - Initializes serial at 115200 baud

- **`uart_ctrl.h`**: Serial communication implementation
  - `serialCtrl()`: Reads and buffers serial data
  - `jsonCmdReceiveHandler()`: Processes commands

- **`json_cmd.h`**: Command type definitions
  - All `CMD_*` and `FEEDBACK_*` constants
  - Command value ranges and meanings

### Response Generation Files

- **`ugv_advance.h`**: Base feedback
  - `baseInfoFeedback()`: Generates T=1001 responses

- **`IMU_ctrl.h`**: IMU data
  - `getIMUData()`: Generates T=1002 responses

- **`gimbal_module.h`**: Gimbal feedback
  - Error feedback (T=1005) generation

- **`RoArm-M2_module.h`**: Robot arm feedback
  - Arm position and status feedback

### Configuration Files

- **`ugv_config.h`**: System configuration
  - Default values, servo IDs, module types
  - Feedback control flags

---

## Protocol Best Practices

### Sending Commands

1. **Always terminate with newline**: `\n` character required
2. **Valid JSON**: Ensure proper JSON formatting
3. **Required fields**: Include all required parameters
4. **Type matching**: Use correct data types (int vs float)
5. **Wait for responses**: Allow time for ESP32 to process

### Receiving Responses

1. **Parse JSON**: Use JSON parser, don't parse manually
2. **Check T value**: Verify response type matches request
3. **Handle timeouts**: ESP32 may not respond immediately
4. **Filter messages**: ESP32 may send unsolicited feedback
5. **Buffer management**: Handle incomplete messages

### Error Handling

1. **Parse errors**: Check JSON validity before processing
2. **Missing fields**: Use default values or skip processing
3. **Type mismatches**: Verify data types match expectations
4. **Timeout handling**: Implement timeout for command responses
5. **Connection loss**: Detect serial port disconnection

---

## Protocol Limitations

1. **No message acknowledgment**: Commands don't confirm receipt
2. **No sequence numbers**: Can't detect lost messages
3. **No checksums**: No data integrity verification
4. **String buffer**: Limited by available RAM
5. **Blocking serial**: `Serial.println()` blocks until sent
6. **No flow control**: No XON/XOFF or RTS/CTS
7. **Single connection**: Only one host can communicate at a time

---

## Troubleshooting

### No Response from ESP32

- Check baud rate (115200)
- Verify serial port connection
- Ensure ESP32 is powered and running
- Check for other programs using the port
- Verify JSON format is correct

### Incomplete Messages

- Ensure newline termination
- Check buffer size limits
- Verify JSON is complete before sending
- Wait for previous command to complete

### Parse Errors

- Validate JSON syntax
- Check for special characters
- Ensure proper escaping of strings
- Verify numeric types (no quotes around numbers)

### Wrong Response Type

- Filter by T value
- Handle multiple response types
- Account for periodic feedback
- Clear buffer before sending command

---

## Conclusion

The ESP32 pan_tilt_base_v0.9 firmware implements a comprehensive JSON-based serial communication protocol that supports:

- **100+ command types** across 11 categories
- **Bidirectional communication** for control and feedback
- **Real-time sensor data** (IMU, servos, battery)
- **File system operations** for mission storage
- **Network configuration** (WiFi, ESP-NOW)
- **Flexible control** for gimbal, robot arm, and base motion

The protocol is designed for flexibility and ease of use, making it suitable for both manual control and automated scripting.

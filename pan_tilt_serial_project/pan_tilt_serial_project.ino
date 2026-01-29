#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <SCServo.h>
#include <INA219_WE.h>
#include "IMU.h"

// ----------------------------- Config -----------------------------
static const uint32_t SERIAL_BAUD = 921600;
static const uint32_t SERVO_BAUD = 1000000;

static const uint8_t S_SCL = 33;
static const uint8_t S_SDA = 32;

static const uint8_t SERVO_RXD = 18;
static const uint8_t SERVO_TXD = 19;

// Servo IDs - confirmed: ID 1 = Pan, ID 2 = Tilt
static const uint8_t PAN_SERVO_ID = 1;
static const uint8_t TILT_SERVO_ID = 2;

static const float PAN_MIN_DEG = -180.0f;
static const float PAN_MAX_DEG = 180.0f;
static const float TILT_MIN_DEG = -90.0f;
static const float TILT_MAX_DEG = 120.0f;

static const uint16_t SERVO_CENTER = 2048;  // Changed from 2047 to 2048 (true center of 0-4095 range)
static const float DEG_TO_POS = 4095.0f / 360.0f;

static const uint32_t IMU_UPDATE_MS = 10;
static const uint32_t INA_UPDATE_MS = 50;
static const uint32_t SERVO_FB_MS = 50;

static const uint32_t DEFAULT_FEEDBACK_MS = 100;
static const uint32_t DEFAULT_HEARTBEAT_MS = 0;  // 0 = disabled (full servo test mode)
static const uint32_t DEFAULT_IDLE_LOCK_MS = 500;

// ----------------------------- Protocol -----------------------------
static const int CMD_PAN_TILT_ABS = 133;
static const int CMD_PAN_TILT_MOVE = 134;
static const int CMD_PAN_TILT_STOP = 135;
static const int CMD_USER_CTRL = 141;

static const int CMD_PAN_LOCK = 170;
static const int CMD_TILT_LOCK = 171;
static const int CMD_PAN_ONLY_ABS = 172;
static const int CMD_TILT_ONLY_ABS = 173;
static const int CMD_PAN_ONLY_MOVE = 174;
static const int CMD_TILT_ONLY_MOVE = 175;

static const int CMD_GET_IMU = 126;
static const int CMD_GET_INA = 160;
static const int CMD_FEEDBACK_FLOW = 131;
static const int CMD_FEEDBACK_INTERVAL = 142;
static const int CMD_HEARTBEAT_SET = 136;
static const int CMD_PING_SERVO = 200;  // Test servo ID
static const int CMD_SET_SERVO_ID = 501;  // Set servo ID: {"T":501,"from":1,"to":2}
static const int CMD_READ_BYTE = 210;   // {"T":210,"id":1,"addr":40} -> T=2101
static const int CMD_WRITE_BYTE = 211;  // {"T":211,"id":1,"addr":40,"value":1}
static const int CMD_READ_WORD = 212;   // {"T":212,"id":1,"addr":42} -> T=2121
static const int CMD_WRITE_WORD = 213;  // {"T":213,"id":1,"addr":48,"value":1000}
static const int CMD_CALIBRATE = 502;   // {"T":502,"id":1} set current pos as middle

static const int FEEDBACK_IMU = 1002;
static const int FEEDBACK_INA = 1010;
static const int FEEDBACK_SERVO = 1011;
static const int FEEDBACK_HEARTBEAT = 1012;

// ----------------------------- Globals -----------------------------
StaticJsonDocument<256> jsonCmdReceive;
StaticJsonDocument<512> jsonOut;

String serialBuffer;

SMS_STS st;

struct ServoFeedback {
  bool ok;
  int pos;
  int speed;
  int load;
  float voltage;
  float current;
  float temp;
  byte mode;
};

ServoFeedback panFb{};
ServoFeedback tiltFb{};

INA219_WE ina219(0x42);
float shuntVoltage_mV = 0.0f;
float loadVoltage_V = 0.0f;
float busVoltage_V = 0.0f;
float current_mA = 0.0f;
float power_mW = 0.0f;
bool ina219_overflow = false;

EulerAngles imuAngles{};
IMU_ST_SENSOR_DATA_FLOAT imuGyro{};
IMU_ST_SENSOR_DATA_FLOAT imuAccel{};
IMU_ST_SENSOR_DATA imuMagn{};
float imuTemp = 0.0f;

float panTargetDeg = 0.0f;
float tiltTargetDeg = 0.0f;
bool panLocked = true;
bool tiltLocked = true;

bool feedbackEnabled = false;
uint32_t feedbackIntervalMs = DEFAULT_FEEDBACK_MS;

uint32_t heartbeatTimeoutMs = DEFAULT_HEARTBEAT_MS;
uint32_t idleLockMs = DEFAULT_IDLE_LOCK_MS;
uint32_t lastCmdRecvTime = 0;
bool heartbeatStopActive = false;

uint32_t lastImuMs = 0;
uint32_t lastInaMs = 0;
uint32_t lastServoFbMs = 0;
uint32_t lastFeedbackMs = 0;
uint32_t lastPanCmdMs = 0;
uint32_t lastTiltCmdMs = 0;

// ----------------------------- Helpers -----------------------------
static float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

static uint16_t panDegToPos(float deg) {
  deg = clampFloat(deg, PAN_MIN_DEG, PAN_MAX_DEG);
  return static_cast<uint16_t>(lroundf(SERVO_CENTER + (deg * DEG_TO_POS)));
}

static uint16_t tiltDegToPos(float deg) {
  deg = clampFloat(deg, TILT_MIN_DEG, TILT_MAX_DEG);
  return static_cast<uint16_t>(lroundf(SERVO_CENTER - (deg * DEG_TO_POS)));
}

static void touchHeartbeat() {
  lastCmdRecvTime = millis();
  heartbeatStopActive = false;
}

static bool readServoFeedback(uint8_t id, ServoFeedback &out) {
  // Add small delay to avoid bus conflicts
  delay(5);
  if (st.FeedBack(id) == -1) {
    out.ok = false;
    return false;
  }
  out.ok = true;
  out.pos = st.ReadPos(-1);
  out.speed = st.ReadSpeed(-1);
  out.load = st.ReadLoad(-1);
  out.voltage = st.ReadVoltage(-1);
  out.current = st.ReadCurrent(-1);
  out.temp = st.ReadTemper(-1);
  out.mode = st.ReadMode(id);
  return true;
}

static void lockAxis(uint8_t id, ServoFeedback &fb, bool &lockedFlag) {
  if (lockedFlag) return;
  // Read current position before locking
  if (readServoFeedback(id, fb)) {
    // Only write position if read was successful
    st.EnableTorque(id, 1);
    if (fb.ok && fb.pos > 0 && fb.pos < 4095) {  // Validate position range
      st.WritePosEx(id, fb.pos, 0, 0);
    }
  } else {
    // If read failed, just enable torque without moving
    st.EnableTorque(id, 1);
  }
  lockedFlag = true;
}

static void unlockAxis(uint8_t id, bool &lockedFlag) {
  if (!lockedFlag) return;
  st.EnableTorque(id, 0);
  lockedFlag = false;
}

static void ensurePositionMode(uint8_t id) {
  // Ensure servo is in position control mode (mode = 0)
  // Wheel mode is mode = 1, position mode is mode = 0
  byte currentMode = st.ReadMode(id);
  if (currentMode != 0) {
    st.writeByte(id, SMS_STS_MODE, 0);  // Set to position control mode
    delay(10);
  }
}

// Set torque limit to max (1000). Per pan_tilt_base_v0.9 RoArm-M2: unlock EPROM,
// write torque limit, lock EPROM — required for torque limit to take effect on ST3215.
static void setServoTorqueLimitMax(uint8_t id) {
  if (st.unLockEprom(id) != -1) {
    delay(10);
    st.writeWord(id, SMS_STS_TORQUE_LIMIT_L, 1000);
    delay(10);
    st.LockEprom(id);
    delay(10);
  }
}

// Tilt uses SERVO_CENTER - deg*DEG_TO_POS: 120° -> 683, -90° -> 3072.
// ST3215 EPROM angle limits clamp commanded position; factory limits often block <90°.
// Set tilt servo min/max angle limits so full -90° to 120° range is allowed.
static const uint16_t TILT_POS_MIN = 683;   // position for 120°
static const uint16_t TILT_POS_MAX = 3072;  // position for -90°
static void setTiltServoAngleLimits() {
  if (st.unLockEprom(TILT_SERVO_ID) != -1) {
    delay(10);
    st.writeWord(TILT_SERVO_ID, SMS_STS_MIN_ANGLE_LIMIT_L, TILT_POS_MIN);
    delay(10);
    st.writeWord(TILT_SERVO_ID, SMS_STS_MAX_ANGLE_LIMIT_L, TILT_POS_MAX);
    delay(10);
    st.LockEprom(TILT_SERVO_ID);
    delay(10);
  }
}

static void setPanTiltAbs(float panDeg, float tiltDeg, uint16_t spd, uint16_t acc) {
  uint16_t posPan = panDegToPos(panDeg);
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  uint8_t ids[2] = {PAN_SERVO_ID, TILT_SERVO_ID};
  int16_t pos[2] = {(int16_t)posPan, (int16_t)posTilt};
  uint16_t spdArr[2] = {spd, spd};
  uint8_t accArr[2] = {(uint8_t)acc, (uint8_t)acc};
  // Ensure servos are unlocked
  panLocked = false;
  tiltLocked = false;
  // Ensure servos are in position control mode
  ensurePositionMode(PAN_SERVO_ID);
  ensurePositionMode(TILT_SERVO_ID);
  delay(5);
  // Set torque limit to max (EPROM unlock/lock per v0.9)
  setServoTorqueLimitMax(PAN_SERVO_ID);
  setServoTorqueLimitMax(TILT_SERVO_ID);
  delay(5);
  // Enable torque on both servos
  st.EnableTorque(PAN_SERVO_ID, 1);
  st.EnableTorque(TILT_SERVO_ID, 1);
  delay(5);
  // Sync write to both servos
  st.SyncWritePosEx(ids, 2, pos, spdArr, accArr);
  panTargetDeg = panDeg;
  tiltTargetDeg = tiltDeg;
}

static void setPanTiltMove(float panDeg, float tiltDeg, uint16_t spdPan, uint16_t spdTilt) {
  uint16_t posPan = panDegToPos(panDeg);
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  uint8_t ids[2] = {PAN_SERVO_ID, TILT_SERVO_ID};
  int16_t pos[2] = {(int16_t)posPan, (int16_t)posTilt};
  uint16_t spdArr[2] = {spdPan, spdTilt};
  uint8_t accArr[2] = {0, 0};
  // Ensure servos are in position control mode
  ensurePositionMode(PAN_SERVO_ID);
  ensurePositionMode(TILT_SERVO_ID);
  delay(5);
  // Set torque limit to max (EPROM unlock/lock per v0.9)
  setServoTorqueLimitMax(PAN_SERVO_ID);
  setServoTorqueLimitMax(TILT_SERVO_ID);
  delay(5);
  st.EnableTorque(PAN_SERVO_ID, 1);
  st.EnableTorque(TILT_SERVO_ID, 1);
  delay(5);
  st.SyncWritePosEx(ids, 2, pos, spdArr, accArr);
  panTargetDeg = panDeg;
  tiltTargetDeg = tiltDeg;
  panLocked = false;
  tiltLocked = false;
}

static void setPanAbs(float panDeg, uint16_t spd, uint16_t acc) {
  uint16_t posPan = panDegToPos(panDeg);
  // Only touch pan servo - don't touch tilt at all
  // Ensure servo is unlocked first
  if (panLocked) {
    panLocked = false;
  }
  // Add delay to ensure previous commands are complete
  delay(10);
  // Ensure servo is in position control mode
  ensurePositionMode(PAN_SERVO_ID);
  delay(5);
  // Set torque limit to max (EPROM unlock/lock per v0.9)
  setServoTorqueLimitMax(PAN_SERVO_ID);
  delay(5);
  // Enable torque and move
  st.EnableTorque(PAN_SERVO_ID, 1);
  delay(5);
  // Write position with speed and acceleration
  int result = st.WritePosEx(PAN_SERVO_ID, posPan, spd, acc);
  panTargetDeg = panDeg;
  panLocked = false;
  // Don't touch tilt servo - leave it completely alone
  // tiltLocked stays as is (should be true from setup)
}

static void setTiltAbs(float tiltDeg, uint16_t spd, uint16_t acc) {
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  // Only touch tilt servo - don't touch pan at all
  // Ensure servo is unlocked first
  if (tiltLocked) {
    tiltLocked = false;
  }
  // Add delay to ensure previous commands are complete
  delay(10);
  // Ensure servo is in position control mode
  ensurePositionMode(TILT_SERVO_ID);
  delay(5);
  // Set torque limit to max (EPROM unlock/lock per v0.9)
  setServoTorqueLimitMax(TILT_SERVO_ID);
  delay(5);
  // Enable torque and move
  st.EnableTorque(TILT_SERVO_ID, 1);
  delay(5);
  // Write position with speed and acceleration
  int result = st.WritePosEx(TILT_SERVO_ID, posTilt, spd, acc);
  tiltTargetDeg = tiltDeg;
  tiltLocked = false;
  // Don't touch pan servo - leave it completely alone
  // panLocked stays as is (should be true from setup)
}

static void setPanMove(float panDeg, uint16_t spd) {
  uint16_t posPan = panDegToPos(panDeg);
  // Ensure servo is in position control mode
  ensurePositionMode(PAN_SERVO_ID);
  delay(5);
  setServoTorqueLimitMax(PAN_SERVO_ID);
  delay(5);
  st.EnableTorque(PAN_SERVO_ID, 1);
  delay(5);
  st.WritePosEx(PAN_SERVO_ID, posPan, spd, 0);
  panTargetDeg = panDeg;
  panLocked = false;
  // Don't touch tilt servo - leave it as is
}

static void setTiltMove(float tiltDeg, uint16_t spd) {
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  // Ensure servo is in position control mode
  ensurePositionMode(TILT_SERVO_ID);
  delay(5);
  setServoTorqueLimitMax(TILT_SERVO_ID);
  delay(5);
  st.EnableTorque(TILT_SERVO_ID, 1);
  delay(5);
  st.WritePosEx(TILT_SERVO_ID, posTilt, spd, 0);
  tiltTargetDeg = tiltDeg;
  tiltLocked = false;
  // Don't touch pan servo - leave it as is
}

static void stopPanTilt() {
  st.EnableTorque(PAN_SERVO_ID, 0);
  st.EnableTorque(TILT_SERVO_ID, 0);
  panLocked = false;
  tiltLocked = false;
}

static void updateImu() {
  imuDataGet(&imuAngles, &imuGyro, &imuAccel, &imuMagn);
}

static void updateIna219() {
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V = busVoltage_V + (shuntVoltage_mV / 1000.0f);
  ina219_overflow = ina219.getOverflow();
}

static void sendImuFeedback() {
  jsonOut.clear();
  jsonOut["T"] = FEEDBACK_IMU;
  jsonOut["r"] = imuAngles.roll;
  jsonOut["p"] = imuAngles.pitch;
  jsonOut["y"] = imuAngles.yaw;
  jsonOut["ax"] = imuAccel.X;
  jsonOut["ay"] = imuAccel.Y;
  jsonOut["az"] = imuAccel.Z;
  jsonOut["gx"] = imuGyro.X;
  jsonOut["gy"] = imuGyro.Y;
  jsonOut["gz"] = imuGyro.Z;
  jsonOut["mx"] = imuMagn.s16X;
  jsonOut["my"] = imuMagn.s16Y;
  jsonOut["mz"] = imuMagn.s16Z;
  jsonOut["temp"] = imuTemp;
  String out;
  serializeJson(jsonOut, out);
  Serial.println(out);
}

static void sendInaFeedback() {
  jsonOut.clear();
  jsonOut["T"] = FEEDBACK_INA;
  jsonOut["bus_v"] = busVoltage_V;
  jsonOut["shunt_mv"] = shuntVoltage_mV;
  jsonOut["load_v"] = loadVoltage_V;
  jsonOut["current_ma"] = current_mA;
  jsonOut["power_mw"] = power_mW;
  jsonOut["overflow"] = ina219_overflow ? 1 : 0;
  String out;
  serializeJson(jsonOut, out);
  Serial.println(out);
}

static void sendServoFeedback() {
  // Only read feedback when explicitly requested, not periodically
  // This avoids interference with movement commands
  readServoFeedback(PAN_SERVO_ID, panFb);
  delay(10);  // Small delay between reads to avoid bus conflicts
  readServoFeedback(TILT_SERVO_ID, tiltFb);
  jsonOut.clear();
  jsonOut["T"] = FEEDBACK_SERVO;
  JsonObject pan = jsonOut.createNestedObject("pan");
  pan["id"] = PAN_SERVO_ID;
  pan["pos"] = panFb.pos;
  pan["speed"] = panFb.speed;
  pan["load"] = panFb.load;
  pan["v"] = panFb.voltage;
  pan["temp"] = panFb.temp;
  pan["mode"] = panFb.mode;
  JsonObject tilt = jsonOut.createNestedObject("tilt");
  tilt["id"] = TILT_SERVO_ID;
  tilt["pos"] = tiltFb.pos;
  tilt["speed"] = tiltFb.speed;
  tilt["load"] = tiltFb.load;
  tilt["v"] = tiltFb.voltage;
  tilt["temp"] = tiltFb.temp;
  tilt["mode"] = tiltFb.mode;
  String out;
  serializeJson(jsonOut, out);
  Serial.println(out);
}

static void sendHeartbeatStatus() {
  jsonOut.clear();
  jsonOut["T"] = FEEDBACK_HEARTBEAT;
  jsonOut["alive"] = heartbeatStopActive ? 0 : 1;
  jsonOut["timeout_ms"] = heartbeatTimeoutMs;
  String out;
  serializeJson(jsonOut, out);
  Serial.println(out);
}

static void handleUserCtrl(int x, int y, int spd) {
  const float step = 5.0f;
  if (x == 2 && y == 2) {
    setPanTiltAbs(0.0f, 0.0f, spd, 0);
    return;
  }
  if (x == -1) panTargetDeg -= step;
  if (x == 1) panTargetDeg += step;
  if (y == -1) tiltTargetDeg -= step;
  if (y == 1) tiltTargetDeg += step;

  panTargetDeg = clampFloat(panTargetDeg, PAN_MIN_DEG, PAN_MAX_DEG);
  tiltTargetDeg = clampFloat(tiltTargetDeg, TILT_MIN_DEG, TILT_MAX_DEG);

  if (x != 0 || y != 0) {
    setPanTiltAbs(panTargetDeg, tiltTargetDeg, spd, 0);
  }
  if (x == 0) {
    lockAxis(PAN_SERVO_ID, panFb, panLocked);
  }
  if (y == 0) {
    lockAxis(TILT_SERVO_ID, tiltFb, tiltLocked);
  }
}

static void processCommand() {
  int cmd = jsonCmdReceive["T"] | -1;
  switch (cmd) {
    case CMD_PAN_TILT_ABS: {
      if (!jsonCmdReceive.containsKey("X") || !jsonCmdReceive.containsKey("Y")) return;
      float pan = jsonCmdReceive["X"];
      float tilt = jsonCmdReceive["Y"];
      // High torque defaults: speed=3400 (max), acc=100 (high acceleration)
      uint16_t spd = jsonCmdReceive["SPD"] | 3400;
      uint16_t acc = jsonCmdReceive["ACC"] | 100;
      setPanTiltAbs(pan, tilt, spd, acc);
      lastPanCmdMs = millis();
      lastTiltCmdMs = millis();
      touchHeartbeat();
    } break;
    case CMD_PAN_TILT_MOVE: {
      if (!jsonCmdReceive.containsKey("X") || !jsonCmdReceive.containsKey("Y")) return;
      float pan = jsonCmdReceive["X"];
      float tilt = jsonCmdReceive["Y"];
      // High torque defaults: speed=3400 (max)
      uint16_t spdPan = jsonCmdReceive["SX"] | 3400;
      uint16_t spdTilt = jsonCmdReceive["SY"] | 3400;
      setPanTiltMove(pan, tilt, spdPan, spdTilt);
      lastPanCmdMs = millis();
      lastTiltCmdMs = millis();
      touchHeartbeat();
    } break;
    case CMD_PAN_ONLY_ABS: {
      if (!jsonCmdReceive.containsKey("X")) return;
      float pan = jsonCmdReceive["X"];
      // High torque defaults: speed=3400 (max), acc=100 (high acceleration)
      uint16_t spd = jsonCmdReceive["SPD"] | 3400;
      uint16_t acc = jsonCmdReceive["ACC"] | 100;
      setPanAbs(pan, spd, acc);
      lastPanCmdMs = millis();
      // setPanAbs already enables tilt torque to hold position
      // No need to call lockAxis which reads/writes position
      touchHeartbeat();
    } break;
    case CMD_TILT_ONLY_ABS: {
      if (!jsonCmdReceive.containsKey("Y")) return;
      float tilt = jsonCmdReceive["Y"];
      // High torque defaults: speed=3400 (max), acc=100 (high acceleration)
      uint16_t spd = jsonCmdReceive["SPD"] | 3400;
      uint16_t acc = jsonCmdReceive["ACC"] | 100;
      setTiltAbs(tilt, spd, acc);
      lastTiltCmdMs = millis();
      // setTiltAbs already enables pan torque to hold position
      // No need to call lockAxis which reads/writes position
      touchHeartbeat();
    } break;
    case CMD_PAN_ONLY_MOVE: {
      if (!jsonCmdReceive.containsKey("X")) return;
      float pan = jsonCmdReceive["X"];
      // High torque default: speed=3400 (max)
      uint16_t spd = jsonCmdReceive["SX"] | 3400;
      setPanMove(pan, spd);
      lastPanCmdMs = millis();
      // setPanMove already enables tilt torque to hold position
      touchHeartbeat();
    } break;
    case CMD_TILT_ONLY_MOVE: {
      if (!jsonCmdReceive.containsKey("Y")) return;
      float tilt = jsonCmdReceive["Y"];
      // High torque default: speed=3400 (max)
      uint16_t spd = jsonCmdReceive["SY"] | 3400;
      setTiltMove(tilt, spd);
      lastTiltCmdMs = millis();
      // setTiltMove already enables pan torque to hold position
      touchHeartbeat();
    } break;
    case CMD_PAN_TILT_STOP:
      stopPanTilt();
      touchHeartbeat();
      break;
    case CMD_USER_CTRL: {
      int x = jsonCmdReceive["X"] | 0;
      int y = jsonCmdReceive["Y"] | 0;
      int spd = jsonCmdReceive["SPD"] | 300;
      handleUserCtrl(x, y, spd);
      lastPanCmdMs = millis();
      lastTiltCmdMs = millis();
      touchHeartbeat();
    } break;
    case CMD_PAN_LOCK: {
      int cmdVal = jsonCmdReceive["cmd"] | 0;
      if (cmdVal == 1) {
        lockAxis(PAN_SERVO_ID, panFb, panLocked);
      } else {
        unlockAxis(PAN_SERVO_ID, panLocked);
      }
      touchHeartbeat();
    } break;
    case CMD_TILT_LOCK: {
      int cmdVal = jsonCmdReceive["cmd"] | 0;
      if (cmdVal == 1) {
        lockAxis(TILT_SERVO_ID, tiltFb, tiltLocked);
      } else {
        unlockAxis(TILT_SERVO_ID, tiltLocked);
      }
      touchHeartbeat();
    } break;
    case CMD_GET_IMU:
      sendImuFeedback();
      break;
    case CMD_GET_INA:
      sendInaFeedback();
      break;
    case CMD_FEEDBACK_FLOW:
      feedbackEnabled = (jsonCmdReceive["cmd"] | 0) == 1;
      break;
    case CMD_FEEDBACK_INTERVAL:
      feedbackIntervalMs = jsonCmdReceive["cmd"] | DEFAULT_FEEDBACK_MS;
      break;
    case CMD_HEARTBEAT_SET:
      heartbeatTimeoutMs = jsonCmdReceive["cmd"] | DEFAULT_HEARTBEAT_MS;
      break;
    case CMD_PING_SERVO: {
      // Diagnostic: Ping a servo ID to see if it responds
      int testId = jsonCmdReceive["id"] | 1;
      int result = st.Ping(testId);
      byte mode = st.ReadMode(testId);
      int torqueLimit = st.readWord(testId, SMS_STS_TORQUE_LIMIT_L);
      byte torqueEnable = st.readByte(testId, SMS_STS_TORQUE_ENABLE);
      int pos = st.ReadPos(testId);
      jsonOut.clear();
      jsonOut["T"] = 2001;  // Ping response
      jsonOut["id"] = testId;
      jsonOut["responded"] = (result == testId) ? 1 : 0;
      jsonOut["result"] = result;
      jsonOut["mode"] = mode;  // 0=position, 1=wheel
      jsonOut["torque_limit"] = torqueLimit;
      jsonOut["torque_enable"] = torqueEnable;
      jsonOut["position"] = pos;
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      break;
    }
    case CMD_SET_SERVO_ID: {
      // Change servo ID: {"T":501,"from":1,"to":2}
      // Process: 1) Unlock EPROM, 2) Write new ID, 3) Lock EPROM
      int fromId = jsonCmdReceive["from"] | 1;
      int toId = jsonCmdReceive["to"] | 2;
      
      if (fromId < 1 || fromId > 254 || toId < 1 || toId > 254) {
        jsonOut.clear();
        jsonOut["T"] = 5001;  // Error response
        jsonOut["error"] = "Invalid ID range (1-254)";
        String out;
        serializeJson(jsonOut, out);
        Serial.println(out);
        break;
      }
      
      // Unlock EPROM
      if (st.unLockEprom(fromId) == -1) {
        jsonOut.clear();
        jsonOut["T"] = 5001;
        jsonOut["error"] = "Failed to unlock EPROM";
        String out;
        serializeJson(jsonOut, out);
        Serial.println(out);
        break;
      }
      
      delay(50);  // Small delay after unlock
      
      // Write new ID to address SMS_STS_ID (5)
      // SMS_STS_ID is defined in SCServo library header
      #define SERVO_ID_ADDR 5  // SMS_STS_ID address
      if (st.writeByte(fromId, SERVO_ID_ADDR, toId) == -1) {
        jsonOut.clear();
        jsonOut["T"] = 5001;
        jsonOut["error"] = "Failed to write new ID";
        String out;
        serializeJson(jsonOut, out);
        Serial.println(out);
        break;
      }
      
      delay(50);  // Small delay after write
      
      // Lock EPROM with new ID
      if (st.LockEprom(toId) == -1) {
        jsonOut.clear();
        jsonOut["T"] = 5001;
        jsonOut["error"] = "Failed to lock EPROM";
        String out;
        serializeJson(jsonOut, out);
        Serial.println(out);
        break;
      }
      
      // Success response
      jsonOut.clear();
      jsonOut["T"] = 5002;  // Success response
      jsonOut["from"] = fromId;
      jsonOut["to"] = toId;
      jsonOut["status"] = "success";
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      
      // Verify by pinging new ID
      delay(100);
      int verifyResult = st.Ping(toId);
      jsonOut.clear();
      jsonOut["T"] = 5003;  // Verification response
      jsonOut["id"] = toId;
      jsonOut["verified"] = (verifyResult == toId) ? 1 : 0;
      String out2;
      serializeJson(jsonOut, out2);
      Serial.println(out2);
      break;
    }
    case CMD_READ_BYTE: {
      int id = jsonCmdReceive["id"] | 1;
      int addr = jsonCmdReceive["addr"] | 40;
      int val = st.readByte(id, (uint8_t)addr);
      jsonOut.clear();
      jsonOut["T"] = 2101;
      jsonOut["id"] = id;
      jsonOut["addr"] = addr;
      jsonOut["value"] = val;
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      break;
    }
    case CMD_WRITE_BYTE: {
      int id = jsonCmdReceive["id"] | 1;
      int addr = jsonCmdReceive["addr"] | 40;
      int value = jsonCmdReceive["value"] | 0;
      bool eprom = (addr >= 5 && addr <= 33 && addr != 3 && addr != 4);
      if (eprom) { st.unLockEprom(id); delay(10); }
      int ret = st.writeByte(id, (uint8_t)addr, (uint8_t)(value & 0xFF));
      if (eprom) { delay(10); st.LockEprom(id); }
      jsonOut.clear();
      jsonOut["T"] = 2111;
      jsonOut["id"] = id;
      jsonOut["addr"] = addr;
      jsonOut["ok"] = (ret != -1) ? 1 : 0;
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      break;
    }
    case CMD_READ_WORD: {
      int id = jsonCmdReceive["id"] | 1;
      int addr = jsonCmdReceive["addr"] | 42;
      int val = st.readWord(id, (uint8_t)addr);
      jsonOut.clear();
      jsonOut["T"] = 2121;
      jsonOut["id"] = id;
      jsonOut["addr"] = addr;
      jsonOut["value"] = val;
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      break;
    }
    case CMD_WRITE_WORD: {
      int id = jsonCmdReceive["id"] | 1;
      int addr = jsonCmdReceive["addr"] | 48;
      int value = jsonCmdReceive["value"] | 0;
      bool eprom = (addr >= 5 && addr <= 33 && addr != 3 && addr != 4);
      if (eprom) { st.unLockEprom(id); delay(10); }
      int ret = st.writeWord(id, (uint8_t)addr, (uint16_t)(value & 0xFFFF));
      if (eprom) { delay(10); st.LockEprom(id); }
      jsonOut.clear();
      jsonOut["T"] = 2131;
      jsonOut["id"] = id;
      jsonOut["addr"] = addr;
      jsonOut["ok"] = (ret != -1) ? 1 : 0;
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      break;
    }
    case CMD_CALIBRATE: {
      int id = jsonCmdReceive["id"] | 1;
      int ret = st.writeByte(id, SMS_STS_TORQUE_ENABLE, 128);  // CalibrationOfs
      jsonOut.clear();
      jsonOut["T"] = 5021;
      jsonOut["id"] = id;
      jsonOut["ok"] = (ret != -1) ? 1 : 0;
      String out;
      serializeJson(jsonOut, out);
      Serial.println(out);
      break;
    }
    default:
      break;
  }
}

static void serialCtrl() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      DeserializationError err = deserializeJson(jsonCmdReceive, serialBuffer);
      serialBuffer = "";
      if (err == DeserializationError::Ok) {
        processCommand();
      }
    } else if (c != '\r') {
      serialBuffer += c;
      if (serialBuffer.length() > 512) {
        serialBuffer = "";
      }
    }
  }
}

// ----------------------------- Setup/Loop -----------------------------
void setup() {
  Serial.begin(SERIAL_BAUD);
  Wire.begin(S_SDA, S_SCL);

  Serial1.begin(SERVO_BAUD, SERIAL_8N1, SERVO_RXD, SERVO_TXD);
  st.pSerial = &Serial1;

  ina219.init();
  ina219.setADCMode(INA219_BIT_MODE_9);
  ina219.setPGain(INA219_PG_320);
  ina219.setBusRange(INA219_BRNG_16);
  ina219.setShuntSizeInOhms(0.01f);

  imuInit();
  updateImu();
  updateIna219();

  // Don't lock servos at startup - just enable torque to hold position
  // This avoids reading/writing positions which could cause unwanted movement
  // Ensure servos are in position control mode (not wheel mode)
  ensurePositionMode(PAN_SERVO_ID);
  ensurePositionMode(TILT_SERVO_ID);
  delay(10);
  // Set torque limit to max (EPROM unlock/lock per v0.9)
  setServoTorqueLimitMax(PAN_SERVO_ID);
  setServoTorqueLimitMax(TILT_SERVO_ID);
  // Set tilt servo EPROM angle limits so -90° to 120° is allowed (not clamped by factory limits)
  setTiltServoAngleLimits();
  delay(10);
  // Enable torque on both servos
  st.EnableTorque(PAN_SERVO_ID, 1);
  st.EnableTorque(TILT_SERVO_ID, 1);
  delay(10);
  panLocked = true;  // Mark as locked
  tiltLocked = true; // Mark as locked

  lastCmdRecvTime = millis();
}

void loop() {
  const uint32_t now = millis();

  serialCtrl();

  if (now - lastImuMs >= IMU_UPDATE_MS) {
    updateImu();
    lastImuMs = now;
  }
  if (now - lastInaMs >= INA_UPDATE_MS) {
    updateIna219();
    lastInaMs = now;
  }
  // Disable periodic feedback reads to avoid interference
  // Only read when explicitly requested (via sendServoFeedback)
  // if (now - lastServoFbMs >= SERVO_FB_MS) {
  //   readServoFeedback(PAN_SERVO_ID, panFb);
  //   readServoFeedback(TILT_SERVO_ID, tiltFb);
  //   lastServoFbMs = now;
  // }

  if (heartbeatTimeoutMs > 0 && (now - lastCmdRecvTime) > heartbeatTimeoutMs) {
    if (!heartbeatStopActive) {
      stopPanTilt();
      heartbeatStopActive = true;
      sendHeartbeatStatus();
    }
  }

  if (!heartbeatStopActive) {
    // Auto-lock after idle timeout - but use a simpler approach
    // Just enable torque to hold position, don't read/write position
    if (!panLocked && (now - lastPanCmdMs) > idleLockMs) {
      setServoTorqueLimitMax(PAN_SERVO_ID);
      st.EnableTorque(PAN_SERVO_ID, 1);  // Just enable torque to hold
      panLocked = true;
    }
    if (!tiltLocked && (now - lastTiltCmdMs) > idleLockMs) {
      setServoTorqueLimitMax(TILT_SERVO_ID);
      st.EnableTorque(TILT_SERVO_ID, 1);  // Just enable torque to hold
      tiltLocked = true;
    }
  }

  if (feedbackEnabled && (now - lastFeedbackMs) >= feedbackIntervalMs) {
    sendServoFeedback();
    sendInaFeedback();
    lastFeedbackMs = now;
  }
}

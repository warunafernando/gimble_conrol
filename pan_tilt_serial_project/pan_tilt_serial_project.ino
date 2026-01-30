#include <Arduino.h>
#include <Wire.h>
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

// Binary protocol (GIMBAL_PROTOCOL.md)
static const uint8_t STX = 0x02;
static const uint8_t ETX = 0x03;
static const uint16_t RSP_ACK_RECEIVED = 1;
static const uint16_t RSP_ACK_EXECUTED = 2;
static const uint16_t RSP_NACK = 3;
static const uint16_t CMD_ENTER_TRACKING = 137;
static const uint16_t CMD_ENTER_CONFIG = 139;
static const uint16_t CMD_EXIT_CONFIG = 140;

static const size_t FRAME_BUF_SIZE = 512;
static const size_t MAX_PAYLOAD = 251;

// ----------------------------- State machine -----------------------------
enum GimbalState { IDLE, TRACKING, CONFIG };
static GimbalState gimbalState = IDLE;

// ----------------------------- Globals -----------------------------
static uint8_t frameBuf[FRAME_BUF_SIZE];
static size_t frameLen = 0;

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

static volatile bool controlTick = false;
static uint16_t lastPanPosTracking = SERVO_CENTER;
static uint16_t lastTiltPosTracking = SERVO_CENTER;
static hw_timer_t* controlTimer = nullptr;

// ----------------------------- CRC8 (poly 0x07, init 0x00) -----------------------------
static uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)(0x07 ^ (crc << 1)) : (uint8_t)(crc << 1);
  }
  return crc;
}

// Build and send binary frame: STX LEN SEQ TYPE PAYLOAD CHECKSUM ETX
static void sendFrame(uint16_t seq, uint16_t type, const uint8_t* payload, size_t payloadLen) {
  if (payloadLen > MAX_PAYLOAD) return;
  uint8_t len = (uint8_t)(4 + payloadLen);  // SEQ(2) + TYPE(2) + PAYLOAD
  uint8_t buf[8 + MAX_PAYLOAD];
  size_t i = 0;
  buf[i++] = STX;
  buf[i++] = len;
  buf[i++] = (uint8_t)(seq & 0xFF);
  buf[i++] = (uint8_t)(seq >> 8);
  buf[i++] = (uint8_t)(type & 0xFF);
  buf[i++] = (uint8_t)(type >> 8);
  for (size_t j = 0; j < payloadLen; j++) buf[i++] = payload[j];
  uint8_t* crcStart = &buf[1];
  buf[i++] = crc8(crcStart, (size_t)(len + 1));
  buf[i++] = ETX;
  Serial.write(buf, i);
}

// NACK: code 1=checksum, 2=unknown type, 3=state rejected, 4=execution failed
static void sendNack(uint16_t seq, uint8_t code) {
  uint8_t pl[] = { code };
  sendFrame(seq, RSP_NACK, pl, 1);
}

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

// No-delay path for TRACKING: read load and pos only (for ACK_EXECUTED)
static void readServoFeedbackTracking(uint8_t id, int16_t* load, int16_t* pos) {
  if (st.FeedBack(id) == -1) {
    *load = 0;
    *pos = 0;
    return;
  }
  *load = (int16_t)st.ReadLoad(-1);
  *pos = (int16_t)st.ReadPos(-1);
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

// One-time servo prep on TRACKING entry (no per-move EPROM/delays)
static void onEnterTracking() {
  ensurePositionMode(PAN_SERVO_ID);
  ensurePositionMode(TILT_SERVO_ID);
  delay(5);
  setServoTorqueLimitMax(PAN_SERVO_ID);
  setServoTorqueLimitMax(TILT_SERVO_ID);
  delay(5);
  st.EnableTorque(PAN_SERVO_ID, 1);
  st.EnableTorque(TILT_SERVO_ID, 1);
  panLocked = false;
  tiltLocked = false;
}

// Fast path: no delay, no EPROM, no ensurePositionMode per move
static void setPanTiltAbsTracking(float panDeg, float tiltDeg, uint16_t spd, uint16_t acc) {
  uint16_t posPan = panDegToPos(panDeg);
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  lastPanPosTracking = posPan;
  lastTiltPosTracking = posTilt;
  uint8_t ids[2] = {PAN_SERVO_ID, TILT_SERVO_ID};
  int16_t pos[2] = {(int16_t)posPan, (int16_t)posTilt};
  uint16_t spdArr[2] = {spd, spd};
  uint8_t accArr[2] = {(uint8_t)acc, (uint8_t)acc};
  st.SyncWritePosEx(ids, 2, pos, spdArr, accArr);
  panTargetDeg = panDeg;
  tiltTargetDeg = tiltDeg;
}

static void setPanTiltMoveTracking(float panDeg, float tiltDeg, uint16_t spdPan, uint16_t spdTilt) {
  uint16_t posPan = panDegToPos(panDeg);
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  lastPanPosTracking = posPan;
  lastTiltPosTracking = posTilt;
  uint8_t ids[2] = {PAN_SERVO_ID, TILT_SERVO_ID};
  int16_t pos[2] = {(int16_t)posPan, (int16_t)posTilt};
  uint16_t spdArr[2] = {spdPan, spdTilt};
  uint8_t accArr[2] = {0, 0};
  st.SyncWritePosEx(ids, 2, pos, spdArr, accArr);
  panTargetDeg = panDeg;
  tiltTargetDeg = tiltDeg;
}

static void setPanAbsTracking(float panDeg, uint16_t spd, uint16_t acc) {
  uint16_t posPan = panDegToPos(panDeg);
  lastPanPosTracking = posPan;
  st.WritePosEx(PAN_SERVO_ID, posPan, spd, acc);
  panTargetDeg = panDeg;
}

static void setTiltAbsTracking(float tiltDeg, uint16_t spd, uint16_t acc) {
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  lastTiltPosTracking = posTilt;
  st.WritePosEx(TILT_SERVO_ID, posTilt, spd, acc);
  tiltTargetDeg = tiltDeg;
}

static void setPanMoveTracking(float panDeg, uint16_t spd) {
  uint16_t posPan = panDegToPos(panDeg);
  lastPanPosTracking = posPan;
  st.WritePosEx(PAN_SERVO_ID, posPan, spd, 0);
  panTargetDeg = panDeg;
}

static void setTiltMoveTracking(float tiltDeg, uint16_t spd) {
  uint16_t posTilt = tiltDegToPos(tiltDeg);
  lastTiltPosTracking = posTilt;
  st.WritePosEx(TILT_SERVO_ID, posTilt, spd, 0);
  tiltTargetDeg = tiltDeg;
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

// Binary IMU response: 50 bytes per GIMBAL_PROTOCOL
static void sendImuBinary(uint16_t seq) {
  updateImu();
  uint8_t buf[50];
  size_t i = 0;
  memcpy(&buf[i], &imuAngles.roll, 4); i += 4;
  memcpy(&buf[i], &imuAngles.pitch, 4); i += 4;
  memcpy(&buf[i], &imuAngles.yaw, 4); i += 4;
  memcpy(&buf[i], &imuAccel.X, 4); i += 4;
  memcpy(&buf[i], &imuAccel.Y, 4); i += 4;
  memcpy(&buf[i], &imuAccel.Z, 4); i += 4;
  memcpy(&buf[i], &imuGyro.X, 4); i += 4;
  memcpy(&buf[i], &imuGyro.Y, 4); i += 4;
  memcpy(&buf[i], &imuGyro.Z, 4); i += 4;
  int16_t mx = (int16_t)imuMagn.s16X, my = (int16_t)imuMagn.s16Y, mz = (int16_t)imuMagn.s16Z;
  memcpy(&buf[i], &mx, 2); i += 2;
  memcpy(&buf[i], &my, 2); i += 2;
  memcpy(&buf[i], &mz, 2); i += 2;
  memcpy(&buf[i], &imuTemp, 4); i += 4;
  sendFrame(seq, FEEDBACK_IMU, buf, 50);
}

// Binary INA response: 21 bytes
static void sendInaBinary(uint16_t seq) {
  updateIna219();
  uint8_t buf[21];
  size_t i = 0;
  memcpy(&buf[i], &busVoltage_V, 4); i += 4;
  memcpy(&buf[i], &shuntVoltage_mV, 4); i += 4;
  memcpy(&buf[i], &loadVoltage_V, 4); i += 4;
  memcpy(&buf[i], &current_mA, 4); i += 4;
  memcpy(&buf[i], &power_mW, 4); i += 4;
  buf[i++] = ina219_overflow ? 1 : 0;
  sendFrame(seq, FEEDBACK_INA, buf, 21);
}

static void sendServoFeedbackBinary(uint16_t seq) {
  readServoFeedback(PAN_SERVO_ID, panFb);
  delay(10);
  readServoFeedback(TILT_SERVO_ID, tiltFb);
  uint8_t buf[8];
  buf[0] = (uint8_t)(panFb.pos & 0xFF); buf[1] = (uint8_t)(panFb.pos >> 8);
  buf[2] = (uint8_t)(panFb.load & 0xFF); buf[3] = (uint8_t)(panFb.load >> 8);
  buf[4] = (uint8_t)(tiltFb.pos & 0xFF); buf[5] = (uint8_t)(tiltFb.pos >> 8);
  buf[6] = (uint8_t)(tiltFb.load & 0xFF); buf[7] = (uint8_t)(tiltFb.load >> 8);
  sendFrame(seq, FEEDBACK_SERVO, buf, 8);
}

static void sendHeartbeatBinary(uint16_t seq) {
  uint8_t buf[3];
  buf[0] = heartbeatStopActive ? 0 : 1;
  buf[1] = (uint8_t)(heartbeatTimeoutMs & 0xFF);
  buf[2] = (uint8_t)(heartbeatTimeoutMs >> 8);
  sendFrame(seq, FEEDBACK_HEARTBEAT, buf, 3);
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

// Send ACK_EXECUTED with 8-byte move feedback (pan_load, pan_pos, tilt_load, tilt_pos)
static void sendMoveFeedback(uint16_t seq) {
  int16_t panLoad, panPos, tiltLoad, tiltPos;
  readServoFeedbackTracking(PAN_SERVO_ID, &panLoad, &panPos);
  readServoFeedbackTracking(TILT_SERVO_ID, &tiltLoad, &tiltPos);
  uint8_t buf[8];
  buf[0] = (uint8_t)((uint16_t)panLoad & 0xFF); buf[1] = (uint8_t)((uint16_t)panLoad >> 8);
  buf[2] = (uint8_t)((uint16_t)panPos & 0xFF); buf[3] = (uint8_t)((uint16_t)panPos >> 8);
  buf[4] = (uint8_t)((uint16_t)tiltLoad & 0xFF); buf[5] = (uint8_t)((uint16_t)tiltLoad >> 8);
  buf[6] = (uint8_t)((uint16_t)tiltPos & 0xFF); buf[7] = (uint8_t)((uint16_t)tiltPos >> 8);
  sendFrame(seq, RSP_ACK_EXECUTED, buf, 8);
}

static bool isConfigType(uint16_t type) {
  return type == 200 || type == 501 || type == 210 || type == 211 || type == 212 || type == 213 || type == 502;
}
static bool isMoveType(uint16_t type) {
  return type == 133 || type == 134 || type == 135 || type == 172 || type == 173 || type == 174 || type == 175;
}

static void processCommand(uint16_t seq, uint16_t type, const uint8_t* payload, size_t payloadLen) {
  if (gimbalState == TRACKING && isConfigType(type)) {
    sendNack(seq, 3);
    return;
  }
  if (gimbalState == CONFIG && isMoveType(type)) {
    sendNack(seq, 3);
    return;
  }

  switch (type) {
    case CMD_ENTER_TRACKING:
      gimbalState = TRACKING;
      onEnterTracking();
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;
    case CMD_ENTER_CONFIG:
      gimbalState = CONFIG;
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;
    case CMD_EXIT_CONFIG:
      gimbalState = IDLE;
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;

    case CMD_PAN_TILT_ABS: {
      if (payloadLen < 12) { sendNack(seq, 4); return; }
      float pan, tilt;
      memcpy(&pan, &payload[0], 4); memcpy(&tilt, &payload[4], 4);
      uint16_t spd = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);
      uint16_t acc = (uint16_t)payload[10] | ((uint16_t)payload[11] << 8);
      if (spd == 0) spd = 3400; if (acc == 0) acc = 100;
      lastPanCmdMs = millis(); lastTiltCmdMs = millis(); touchHeartbeat();
      if (gimbalState == TRACKING) {
        setPanTiltAbsTracking(pan, tilt, spd, acc);
        sendMoveFeedback(seq);
      } else {
        setPanTiltAbs(pan, tilt, spd, acc);
        sendMoveFeedback(seq);
      }
    } break;
    case CMD_PAN_TILT_MOVE: {
      if (payloadLen < 12) { sendNack(seq, 4); return; }
      float pan, tilt;
      memcpy(&pan, &payload[0], 4); memcpy(&tilt, &payload[4], 4);
      uint16_t spdPan = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);
      uint16_t spdTilt = (uint16_t)payload[10] | ((uint16_t)payload[11] << 8);
      if (spdPan == 0) spdPan = 3400; if (spdTilt == 0) spdTilt = 3400;
      lastPanCmdMs = millis(); lastTiltCmdMs = millis(); touchHeartbeat();
      if (gimbalState == TRACKING) {
        setPanTiltMoveTracking(pan, tilt, spdPan, spdTilt);
        sendMoveFeedback(seq);
      } else {
        setPanTiltMove(pan, tilt, spdPan, spdTilt);
        sendMoveFeedback(seq);
      }
    } break;
    case CMD_PAN_ONLY_ABS: {
      if (payloadLen < 8) { sendNack(seq, 4); return; }
      float pan; memcpy(&pan, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      uint16_t acc = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
      if (spd == 0) spd = 3400; if (acc == 0) acc = 100;
      lastPanCmdMs = millis(); touchHeartbeat();
      if (gimbalState == TRACKING) {
        setPanAbsTracking(pan, spd, acc);
        sendMoveFeedback(seq);
      } else {
        setPanAbs(pan, spd, acc);
        sendMoveFeedback(seq);
      }
    } break;
    case CMD_TILT_ONLY_ABS: {
      if (payloadLen < 8) { sendNack(seq, 4); return; }
      float tilt; memcpy(&tilt, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      uint16_t acc = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
      if (spd == 0) spd = 3400; if (acc == 0) acc = 100;
      lastTiltCmdMs = millis(); touchHeartbeat();
      if (gimbalState == TRACKING) {
        setTiltAbsTracking(tilt, spd, acc);
        sendMoveFeedback(seq);
      } else {
        setTiltAbs(tilt, spd, acc);
        sendMoveFeedback(seq);
      }
    } break;
    case CMD_PAN_ONLY_MOVE: {
      if (payloadLen < 6) { sendNack(seq, 4); return; }
      float pan; memcpy(&pan, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      if (spd == 0) spd = 3400;
      lastPanCmdMs = millis(); touchHeartbeat();
      if (gimbalState == TRACKING) {
        setPanMoveTracking(pan, spd);
        sendMoveFeedback(seq);
      } else {
        setPanMove(pan, spd);
        sendMoveFeedback(seq);
      }
    } break;
    case CMD_TILT_ONLY_MOVE: {
      if (payloadLen < 6) { sendNack(seq, 4); return; }
      float tilt; memcpy(&tilt, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      if (spd == 0) spd = 3400;
      lastTiltCmdMs = millis(); touchHeartbeat();
      if (gimbalState == TRACKING) {
        setTiltMoveTracking(tilt, spd);
        sendMoveFeedback(seq);
      } else {
        setTiltMove(tilt, spd);
        sendMoveFeedback(seq);
      }
    } break;
    case CMD_PAN_TILT_STOP:
      if (gimbalState == TRACKING) gimbalState = IDLE;
      stopPanTilt();
      touchHeartbeat();
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;
    case CMD_USER_CTRL: {
      if (payloadLen < 4) { sendNack(seq, 4); return; }
      int8_t x = (int8_t)payload[0], y = (int8_t)payload[1];
      uint16_t spd = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
      if (spd == 0) spd = 300;
      lastPanCmdMs = millis(); lastTiltCmdMs = millis(); touchHeartbeat();
      handleUserCtrl(x, y, spd);
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
    } break;
    case CMD_PAN_LOCK: {
      if (payloadLen < 1) { sendNack(seq, 4); return; }
      if (payload[0] == 1) lockAxis(PAN_SERVO_ID, panFb, panLocked);
      else unlockAxis(PAN_SERVO_ID, panLocked);
      touchHeartbeat();
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
    } break;
    case CMD_TILT_LOCK: {
      if (payloadLen < 1) { sendNack(seq, 4); return; }
      if (payload[0] == 1) lockAxis(TILT_SERVO_ID, tiltFb, tiltLocked);
      else unlockAxis(TILT_SERVO_ID, tiltLocked);
      touchHeartbeat();
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
    } break;
    case CMD_GET_IMU:
      sendImuBinary(seq);
      break;
    case CMD_GET_INA:
      sendInaBinary(seq);
      break;
    case CMD_FEEDBACK_FLOW:
      if (payloadLen >= 1) feedbackEnabled = (payload[0] == 1);
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;
    case CMD_FEEDBACK_INTERVAL:
      if (payloadLen >= 2) feedbackIntervalMs = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
      if (feedbackIntervalMs == 0) feedbackIntervalMs = DEFAULT_FEEDBACK_MS;
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;
    case CMD_HEARTBEAT_SET:
      if (payloadLen >= 2) heartbeatTimeoutMs = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
      break;
    case CMD_PING_SERVO: {
      if (payloadLen < 1) { sendNack(seq, 4); return; }
      uint8_t testId = payload[0]; if (testId == 0) testId = 1;
      int result = st.Ping(testId);
      byte mode = st.ReadMode(testId);
      int torqueLimit = st.readWord(testId, SMS_STS_TORQUE_LIMIT_L);
      byte torqueEnable = st.readByte(testId, SMS_STS_TORQUE_ENABLE);
      int pos = st.ReadPos(testId);
      uint8_t buf[9];
      buf[0] = testId;
      buf[1] = (result == (int)testId) ? 1 : 0;
      buf[2] = (uint8_t)(result & 0xFF);
      buf[3] = mode;
      buf[4] = (uint8_t)(torqueLimit & 0xFF); buf[5] = (uint8_t)(torqueLimit >> 8);
      buf[6] = torqueEnable;
      buf[7] = (uint8_t)(pos & 0xFF); buf[8] = (uint8_t)(pos >> 8);
      sendFrame(seq, (uint16_t)2001, buf, 9);
    } break;
    case CMD_SET_SERVO_ID: {
      if (payloadLen < 2) { sendNack(seq, 4); return; }
      uint8_t fromId = payload[0], toId = payload[1];
      if (fromId < 1 || fromId > 254 || toId < 1 || toId > 254) {
        sendNack(seq, 4);
        return;
      }
      if (st.unLockEprom(fromId) == -1) { sendNack(seq, 4); return; }
      delay(50);
      #define SERVO_ID_ADDR 5
      if (st.writeByte(fromId, SERVO_ID_ADDR, toId) == -1) { sendNack(seq, 4); return; }
      delay(50);
      if (st.LockEprom(toId) == -1) { sendNack(seq, 4); return; }
      uint8_t okBuf[2] = { fromId, toId };
      sendFrame(seq, (uint16_t)5002, okBuf, 2);
      delay(100);
      int verifyResult = st.Ping(toId);
      uint8_t verifyBuf[2] = { (uint8_t)toId, (uint8_t)((verifyResult == (int)toId) ? 1 : 0) };
      sendFrame(seq, (uint16_t)5003, verifyBuf, 2);
    } break;
    case CMD_READ_BYTE: {
      if (payloadLen < 2) { sendNack(seq, 4); return; }
      uint8_t id = payload[0], addr = payload[1];
      int val = st.readByte(id, addr);
      uint8_t buf[3] = { id, addr, (uint8_t)(val & 0xFF) };
      sendFrame(seq, (uint16_t)2101, buf, 3);
    } break;
    case CMD_WRITE_BYTE: {
      if (payloadLen < 3) { sendNack(seq, 4); return; }
      uint8_t id = payload[0], addr = payload[1], value = payload[2];
      bool eprom = (addr >= 5 && addr <= 33 && addr != 3 && addr != 4);
      if (eprom) { st.unLockEprom(id); delay(10); }
      int ret = st.writeByte(id, addr, value);
      if (eprom) { delay(10); st.LockEprom(id); }
      uint8_t buf[3] = { id, addr, (uint8_t)((ret != -1) ? 1 : 0) };
      sendFrame(seq, (uint16_t)2111, buf, 3);
    } break;
    case CMD_READ_WORD: {
      if (payloadLen < 2) { sendNack(seq, 4); return; }
      uint8_t id = payload[0], addr = payload[1];
      int val = st.readWord(id, addr);
      uint8_t buf[4];
      buf[0] = id; buf[1] = addr; buf[2] = (uint8_t)(val & 0xFF); buf[3] = (uint8_t)(val >> 8);
      sendFrame(seq, (uint16_t)2121, buf, 4);
    } break;
    case CMD_WRITE_WORD: {
      if (payloadLen < 4) { sendNack(seq, 4); return; }
      uint8_t id = payload[0], addr = payload[1];
      uint16_t value = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
      bool eprom = (addr >= 5 && addr <= 33 && addr != 3 && addr != 4);
      if (eprom) { st.unLockEprom(id); delay(10); }
      int ret = st.writeWord(id, addr, value);
      if (eprom) { delay(10); st.LockEprom(id); }
      uint8_t buf[3] = { id, addr, (uint8_t)((ret != -1) ? 1 : 0) };
      sendFrame(seq, (uint16_t)2131, buf, 3);
    } break;
    case CMD_CALIBRATE: {
      if (payloadLen < 1) { sendNack(seq, 4); return; }
      uint8_t id = payload[0]; if (id == 0) id = 1;
      int ret = st.writeByte(id, SMS_STS_TORQUE_ENABLE, 128);
      uint8_t buf[2] = { id, (uint8_t)((ret != -1) ? 1 : 0) };
      sendFrame(seq, (uint16_t)5021, buf, 2);
    } break;
    default:
      sendNack(seq, 2);
      break;
  }
}

// Non-blocking binary frame parser: STX LEN SEQ TYPE PAYLOAD CHECKSUM ETX
static void serialCtrl() {
  while (Serial.available() > 0 && frameLen < FRAME_BUF_SIZE) {
    frameBuf[frameLen++] = (uint8_t)Serial.read();
  }
  if (frameLen < 8) return;
  size_t stxIdx = 0;
  for (; stxIdx < frameLen && frameBuf[stxIdx] != STX; stxIdx++) {}
  if (stxIdx >= frameLen) {
    frameLen = 0;
    return;
  }
  if (stxIdx > 0) {
    memmove(frameBuf, &frameBuf[stxIdx], frameLen - stxIdx);
    frameLen -= stxIdx;
  }
  uint8_t len = frameBuf[1];
  if (len < 4 || len > MAX_PAYLOAD) {
    frameLen = 0;
    return;
  }
  size_t frameSize = (size_t)(4 + len);
  if (frameLen < frameSize) return;
  uint8_t recvCrc = frameBuf[2 + len];
  if (frameBuf[3 + len] != ETX) {
    frameLen = 0;
    return;
  }
  uint8_t compCrc = crc8(&frameBuf[1], (size_t)(len + 1));
  if (recvCrc != compCrc) {
    uint16_t seq = (uint16_t)frameBuf[2] | ((uint16_t)frameBuf[3] << 8);
    sendNack(seq, 1);
    memmove(frameBuf, &frameBuf[frameSize], frameLen - frameSize);
    frameLen -= frameSize;
    return;
  }
  uint16_t seq = (uint16_t)frameBuf[2] | ((uint16_t)frameBuf[3] << 8);
  uint16_t type = (uint16_t)frameBuf[4] | ((uint16_t)frameBuf[5] << 8);
  const uint8_t* payload = (len > 4) ? &frameBuf[6] : nullptr;
  size_t payloadLen = (size_t)(len - 4);
  sendFrame(seq, RSP_ACK_RECEIVED, nullptr, 0);
  processCommand(seq, type, payload, payloadLen);
  memmove(frameBuf, &frameBuf[frameSize], frameLen - frameSize);
  frameLen -= frameSize;
}

void IRAM_ATTR onControlTimer() {
  controlTick = true;
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

  controlTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(controlTimer, &onControlTimer, true);
  timerAlarmWrite(controlTimer, 10000, true);
  timerAlarmEnable(controlTimer);
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
      sendHeartbeatBinary(0);
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
    sendServoFeedbackBinary(0);
    sendInaBinary(0);
    lastFeedbackMs = now;
  }
}

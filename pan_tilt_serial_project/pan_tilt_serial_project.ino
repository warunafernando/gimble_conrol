#include <Arduino.h>
#include <Wire.h>
#include <SCServo.h>
#include <INA219_WE.h>
#include "IMU.h"

// OTA includes
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_system.h>

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

// ----------------------------- Safety Configuration -----------------------------
// Stall detection: load threshold and time before emergency stop
static const int16_t STALL_LOAD_THRESHOLD = 900;      // Load units (max ~1000)
static const uint32_t STALL_TIME_THRESHOLD_MS = 500;  // Time at high load before stop
static const uint32_t STALL_CHECK_INTERVAL_MS = 50;   // How often to check for stall

// Current limit: maximum allowed bus current (mA)
static const float MAX_BUS_CURRENT_MA = 3000.0f;      // 3A limit

// Servo mode verification interval
static const uint32_t MODE_CHECK_INTERVAL_MS = 1000;  // Check every 1 second

// Temperature limits (Celsius)
static const float SERVO_TEMP_WARNING = 55.0f;
static const float SERVO_TEMP_CRITICAL = 65.0f;

// Speed/acceleration limits
static const uint16_t MAX_SERVO_SPEED = 4000;     // Maximum allowed speed
static const uint16_t MAX_SERVO_ACCEL = 254;      // Maximum allowed acceleration
static const uint16_t THERMAL_THROTTLE_SPEED = 1500;  // Reduced speed when thermal throttling

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
static const int FEEDBACK_STATE = 1013;

// Binary protocol (GIMBAL_PROTOCOL.md)
static const uint8_t STX = 0x02;
static const uint8_t ETX = 0x03;
static const uint16_t RSP_ACK_RECEIVED = 1;
static const uint16_t RSP_ACK_EXECUTED = 2;
static const uint16_t RSP_NACK = 3;
static const uint16_t CMD_ENTER_TRACKING = 137;
static const uint16_t CMD_ENTER_CONFIG = 139;
static const uint16_t CMD_EXIT_CONFIG = 140;
static const uint16_t CMD_GET_STATE = 144;
static const uint16_t CMD_RESET_SAFETY = 145;  // Reset safety stop state
static const uint16_t CMD_GET_SAFETY_STATUS = 146;  // Get safety status

// FW version: from build flag or default
#ifndef APP_VERSION
#define APP_VERSION "1.0.0"
#endif
static const char FW_VERSION[] = APP_VERSION;

// OTA commands (UART firmware update)
static const uint16_t CMD_OTA_START = 600;
static const uint16_t CMD_OTA_CHUNK = 601;
static const uint16_t CMD_OTA_END = 602;
static const uint16_t CMD_OTA_ABORT = 603;

// FW info commands
static const uint16_t CMD_GET_FW_INFO = 610;
static const uint16_t CMD_SWITCH_FW = 611;

// OTA responses
static const uint16_t RSP_OTA_STARTED = 2600;
static const uint16_t RSP_OTA_CHUNK = 2601;
static const uint16_t RSP_OTA_DONE = 2602;
static const uint16_t RSP_OTA_NACK = 2603;
static const uint16_t RSP_FW_INFO = 2610;

static const size_t FW_VERSION_LEN = 32;  // Max version string length in payload

// OTA hash types
static const uint8_t OTA_HASH_NONE = 0;
static const uint8_t OTA_HASH_CRC32 = 1;
static const uint8_t OTA_HASH_SHA256 = 2;

// OTA error codes
static const uint8_t OTA_ERR_SIZE_MISMATCH = 1;
static const uint8_t OTA_ERR_CHECKSUM_FAIL = 2;
static const uint8_t OTA_ERR_FLASH_ERROR = 3;
static const uint8_t OTA_ERR_TIMEOUT = 4;
static const uint8_t OTA_ERR_ABORTED = 5;

static const size_t FRAME_BUF_SIZE = 512;
static const size_t MAX_PAYLOAD = 251;

// ----------------------------- State machine -----------------------------
enum GimbalState { IDLE, TRACKING, CONFIG, OTA_RECEIVE };
static GimbalState gimbalState = IDLE;

// ----------------------------- OTA globals -----------------------------
static esp_ota_handle_t otaHandle = 0;
static const esp_partition_t* otaPartition = nullptr;
static uint32_t otaTotalSize = 0;
static uint32_t otaBytesWritten = 0;
static uint8_t otaHashType = OTA_HASH_NONE;
static uint32_t otaExpectedCrc32 = 0;
static uint8_t otaExpectedSha256[32] = {0};
static uint32_t otaRunningCrc32 = 0;
static uint32_t otaLastChunkTime = 0;
static bool otaInProgress = false;  // Flag to track if OTA is actually in progress
static const uint32_t OTA_TIMEOUT_MS = 60000;  // 60 second timeout

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

// ----------------------------- Safety State -----------------------------
static bool safetyStopTriggered = false;
static uint8_t safetyStopReason = 0;
static uint32_t lastStallCheckMs = 0;
static uint32_t lastModeCheckMs = 0;
static uint32_t panStallStartMs = 0;
static uint32_t tiltStallStartMs = 0;
static bool panInStall = false;
static bool tiltInStall = false;
static bool thermalThrottleActive = false;

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

// ----------------------------- CRC32 for OTA -----------------------------
static const uint32_t crc32_table[256] = {
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f,
  0xe963a535, 0x9e6495a3, 0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,
  0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 0x1db71064, 0x6ab020f2,
  0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9,
  0xfa0f3d63, 0x8d080df5, 0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,
  0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 0x35b5a8fa, 0x42b2986c,
  0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,
  0xcfba9599, 0xb8bda50f, 0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,
  0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 0x76dc4190, 0x01db7106,
  0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,
  0x91646c97, 0xe6635c01, 0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,
  0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 0x65b0d9c6, 0x12b7e950,
  0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,
  0xa4d1c46d, 0xd3d6f4fb, 0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,
  0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 0x5005713c, 0x270241aa,
  0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,
  0xb7bd5c3b, 0xc0ba6cad, 0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,
  0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 0xe3630b12, 0x94643b84,
  0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,
  0x196c3671, 0x6e6b06e7, 0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,
  0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 0xd6d6a3e8, 0xa1d1937e,
  0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,
  0x316e8eef, 0x4669be79, 0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,
  0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 0xc5ba3bbe, 0xb2bd0b28,
  0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,
  0x72076785, 0x05005713, 0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,
  0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 0x86d3d2d4, 0xf1d4e242,
  0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,
  0x616bffd3, 0x166ccf45, 0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,
  0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 0xaed16a4a, 0xd9d65adc,
  0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,
  0x54de5729, 0x23d967bf, 0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,
  0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

// CRC32 update - does NOT invert input/output, just updates running CRC
// Initialize with 0xFFFFFFFF, finalize with XOR 0xFFFFFFFF
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  while (len--) {
    crc = crc32_table[(crc ^ *data++) & 0xFF] ^ (crc >> 8);
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
// Safety NACK codes: 10=stall, 11=overcurrent, 12=mode_corruption, 13=thermal
static const uint8_t SAFETY_ERR_STALL = 10;
static const uint8_t SAFETY_ERR_OVERCURRENT = 11;
static const uint8_t SAFETY_ERR_MODE_CORRUPTION = 12;
static const uint8_t SAFETY_ERR_THERMAL = 13;

// Safety alert response type
static const uint16_t RSP_SAFETY_ALERT = 3000;

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

// SAFETY: Validate and limit speed/acceleration values
static uint16_t validateSpeed(uint16_t spd) {
  if (spd == 0) spd = 3400;  // Default
  if (spd > MAX_SERVO_SPEED) spd = MAX_SERVO_SPEED;
  // Apply thermal throttling if active
  if (thermalThrottleActive && spd > THERMAL_THROTTLE_SPEED) {
    spd = THERMAL_THROTTLE_SPEED;
  }
  return spd;
}

static uint16_t validateAccel(uint16_t acc) {
  if (acc == 0) acc = 100;  // Default
  if (acc > MAX_SERVO_ACCEL) acc = MAX_SERVO_ACCEL;
  return acc;
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

// SAFETY FIX 1.1: Set pan servo EPROM angle limits (hardware protection)
// Pan uses SERVO_CENTER + deg*DEG_TO_POS: -180° -> 0, +180° -> 4095
static const uint16_t PAN_POS_MIN = 0;      // position for -180°
static const uint16_t PAN_POS_MAX = 4095;   // position for +180°
static void setPanServoAngleLimits() {
  if (st.unLockEprom(PAN_SERVO_ID) != -1) {
    delay(10);
    st.writeWord(PAN_SERVO_ID, SMS_STS_MIN_ANGLE_LIMIT_L, PAN_POS_MIN);
    delay(10);
    st.writeWord(PAN_SERVO_ID, SMS_STS_MAX_ANGLE_LIMIT_L, PAN_POS_MAX);
    delay(10);
    st.LockEprom(PAN_SERVO_ID);
    delay(10);
  }
}

// ----------------------------- Safety Functions -----------------------------

// Send safety alert to host
static void sendSafetyAlert(uint8_t alertCode, const char* message) {
  uint8_t buf[33];
  buf[0] = alertCode;
  size_t msgLen = strlen(message);
  if (msgLen > 31) msgLen = 31;
  memcpy(&buf[1], message, msgLen);
  buf[1 + msgLen] = '\0';
  sendFrame(0, RSP_SAFETY_ALERT, buf, 1 + msgLen + 1);
}

// Emergency stop - disable all servos immediately
static void emergencyStop(uint8_t reason) {
  // Disable torque on both servos
  st.EnableTorque(PAN_SERVO_ID, 0);
  st.EnableTorque(TILT_SERVO_ID, 0);

  // Update state
  panLocked = false;
  tiltLocked = false;
  safetyStopTriggered = true;
  safetyStopReason = reason;

  // Return to IDLE state
  if (gimbalState == TRACKING) {
    gimbalState = IDLE;
  }

  // Send alert
  const char* msg = "Unknown";
  switch (reason) {
    case SAFETY_ERR_STALL: msg = "Stall detected"; break;
    case SAFETY_ERR_OVERCURRENT: msg = "Overcurrent"; break;
    case SAFETY_ERR_MODE_CORRUPTION: msg = "Mode corruption"; break;
    case SAFETY_ERR_THERMAL: msg = "Thermal critical"; break;
  }
  sendSafetyAlert(reason, msg);
}

// SAFETY FIX 5.1: Check for servo stall condition
static void checkServoStall() {
  uint32_t now = millis();
  if (now - lastStallCheckMs < STALL_CHECK_INTERVAL_MS) return;
  lastStallCheckMs = now;

  // Only check when servos are active (not in safety stop)
  if (safetyStopTriggered) return;

  // Read servo loads
  int16_t panLoad = 0, panPos = 0, tiltLoad = 0, tiltPos = 0;
  readServoFeedbackTracking(PAN_SERVO_ID, &panLoad, &panPos);
  readServoFeedbackTracking(TILT_SERVO_ID, &tiltLoad, &tiltPos);

  // Check pan servo stall
  if (abs(panLoad) > STALL_LOAD_THRESHOLD) {
    if (!panInStall) {
      panInStall = true;
      panStallStartMs = now;
    } else if (now - panStallStartMs > STALL_TIME_THRESHOLD_MS) {
      emergencyStop(SAFETY_ERR_STALL);
      return;
    }
  } else {
    panInStall = false;
  }

  // Check tilt servo stall
  if (abs(tiltLoad) > STALL_LOAD_THRESHOLD) {
    if (!tiltInStall) {
      tiltInStall = true;
      tiltStallStartMs = now;
    } else if (now - tiltStallStartMs > STALL_TIME_THRESHOLD_MS) {
      emergencyStop(SAFETY_ERR_STALL);
      return;
    }
  } else {
    tiltInStall = false;
  }
}

// SAFETY FIX 5.2: Check bus current limit
static void checkBusCurrent() {
  // Only check when not already in safety stop
  if (safetyStopTriggered) return;

  if (current_mA > MAX_BUS_CURRENT_MA) {
    emergencyStop(SAFETY_ERR_OVERCURRENT);
  }
}

// SAFETY FIX 2.1: Verify servos are in position mode (not wheel mode)
static void verifyServoModes() {
  uint32_t now = millis();
  if (now - lastModeCheckMs < MODE_CHECK_INTERVAL_MS) return;
  lastModeCheckMs = now;

  // Only check when not already in safety stop
  if (safetyStopTriggered) return;

  byte panMode = st.ReadMode(PAN_SERVO_ID);
  byte tiltMode = st.ReadMode(TILT_SERVO_ID);

  // Mode 0 = position control, Mode 1 = wheel mode
  if (panMode != 0 || tiltMode != 0) {
    // Try to fix it first
    if (panMode != 0) {
      st.writeByte(PAN_SERVO_ID, SMS_STS_MODE, 0);
      delay(5);
    }
    if (tiltMode != 0) {
      st.writeByte(TILT_SERVO_ID, SMS_STS_MODE, 0);
      delay(5);
    }

    // Re-check
    panMode = st.ReadMode(PAN_SERVO_ID);
    tiltMode = st.ReadMode(TILT_SERVO_ID);

    // If still not in position mode, emergency stop
    if (panMode != 0 || tiltMode != 0) {
      emergencyStop(SAFETY_ERR_MODE_CORRUPTION);
    }
  }
}

// SAFETY FIX 9.1: Check servo temperatures
static void checkServoTemperature() {
  // Only check when not already in safety stop
  if (safetyStopTriggered) return;

  // Read temperatures (requires full feedback read)
  ServoFeedback tempFb;
  float panTemp = 0, tiltTemp = 0;

  if (readServoFeedback(PAN_SERVO_ID, tempFb)) {
    panTemp = tempFb.temp;
  }
  if (readServoFeedback(TILT_SERVO_ID, tempFb)) {
    tiltTemp = tempFb.temp;
  }

  // Critical temperature - emergency stop
  if (panTemp > SERVO_TEMP_CRITICAL || tiltTemp > SERVO_TEMP_CRITICAL) {
    emergencyStop(SAFETY_ERR_THERMAL);
    return;
  }

  // Warning temperature - enable thermal throttle (reduce speed)
  if (panTemp > SERVO_TEMP_WARNING || tiltTemp > SERVO_TEMP_WARNING) {
    if (!thermalThrottleActive) {
      thermalThrottleActive = true;
      sendSafetyAlert(SAFETY_ERR_THERMAL, "Thermal warning - throttling");
    }
  } else {
    thermalThrottleActive = false;
  }
}

// Reset safety state (allow operation after safety stop)
static void resetSafetyState() {
  safetyStopTriggered = false;
  safetyStopReason = 0;
  panInStall = false;
  tiltInStall = false;
  thermalThrottleActive = false;
}

// Run all safety checks
static void runSafetyChecks() {
  checkServoStall();
  checkBusCurrent();
  verifyServoModes();
  // Temperature check is slower, run less frequently
  static uint32_t lastTempCheckMs = 0;
  if (millis() - lastTempCheckMs > 5000) {  // Every 5 seconds
    checkServoTemperature();
    lastTempCheckMs = millis();
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
static bool isOtaType(uint16_t type) {
  return type == CMD_OTA_START || type == CMD_OTA_CHUNK || type == CMD_OTA_END || type == CMD_OTA_ABORT;
}

// ----------------------------- OTA Handlers -----------------------------
static void sendOtaNack(uint16_t seq, uint8_t errorCode) {
  uint8_t buf[1] = { errorCode };
  sendFrame(seq, RSP_OTA_NACK, buf, 1);
}

static void otaAbortCleanup() {
  if (otaHandle != 0) {
    esp_ota_abort(otaHandle);
    otaHandle = 0;
  }
  otaPartition = nullptr;
  otaTotalSize = 0;
  otaBytesWritten = 0;
  otaHashType = OTA_HASH_NONE;
  otaRunningCrc32 = 0xFFFFFFFF;  // Reset to initial value
  otaInProgress = false;  // Clear OTA in progress flag
  gimbalState = IDLE;
}

static void handleOtaStart(uint16_t seq, const uint8_t* payload, size_t payloadLen) {
  // Payload: total_size(4), hash_type(1), expected_hash(4 or 32)
  if (payloadLen < 5) {
    sendOtaNack(seq, OTA_ERR_SIZE_MISMATCH);
    return;
  }
  
  uint32_t totalSize;
  memcpy(&totalSize, payload, 4);
  uint8_t hashType = payload[4];
  
  // Get the next OTA partition (inactive slot)
  otaPartition = esp_ota_get_next_update_partition(NULL);
  if (otaPartition == nullptr) {
    sendOtaNack(seq, OTA_ERR_FLASH_ERROR);
    return;
  }
  
  // Check if image fits in partition
  if (totalSize > otaPartition->size) {
    sendOtaNack(seq, OTA_ERR_SIZE_MISMATCH);
    return;
  }
  
  // Store expected hash
  otaHashType = hashType;
  if (hashType == OTA_HASH_CRC32 && payloadLen >= 9) {
    memcpy(&otaExpectedCrc32, &payload[5], 4);
  } else if (hashType == OTA_HASH_SHA256 && payloadLen >= 37) {
    memcpy(otaExpectedSha256, &payload[5], 32);
  }
  
  // Begin OTA
  esp_err_t err = esp_ota_begin(otaPartition, totalSize, &otaHandle);
  if (err != ESP_OK) {
    sendOtaNack(seq, OTA_ERR_FLASH_ERROR);
    return;
  }
  
  // Initialize state
  otaTotalSize = totalSize;
  otaBytesWritten = 0;
  otaRunningCrc32 = 0xFFFFFFFF;  // Standard CRC-32 initial value
  otaLastChunkTime = millis();
  otaInProgress = true;  // Mark OTA as actually started
  gimbalState = OTA_RECEIVE;
  
  // Send ACK_OTA_STARTED: inactive_slot(1), slot_size(4)
  uint8_t buf[5];
  buf[0] = (otaPartition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) ? 0 : 1;
  memcpy(&buf[1], &otaPartition->size, 4);
  sendFrame(seq, RSP_OTA_STARTED, buf, 5);
}

static void handleOtaChunk(uint16_t seq, const uint8_t* payload, size_t payloadLen) {
  // Payload: offset(4), length(2), data(N)
  if (payloadLen < 6) {
    sendOtaNack(seq, OTA_ERR_SIZE_MISMATCH);
    otaAbortCleanup();
    return;
  }
  
  uint32_t offset;
  uint16_t dataLen;
  memcpy(&offset, payload, 4);
  memcpy(&dataLen, &payload[4], 2);
  
  if (payloadLen < 6 + dataLen) {
    sendOtaNack(seq, OTA_ERR_SIZE_MISMATCH);
    otaAbortCleanup();
    return;
  }
  
  // Check offset matches expected
  if (offset != otaBytesWritten) {
    sendOtaNack(seq, OTA_ERR_SIZE_MISMATCH);
    otaAbortCleanup();
    return;
  }
  
  // Check we won't exceed total size
  if (otaBytesWritten + dataLen > otaTotalSize) {
    sendOtaNack(seq, OTA_ERR_SIZE_MISMATCH);
    otaAbortCleanup();
    return;
  }
  
  const uint8_t* data = &payload[6];
  
  // Write chunk
  esp_err_t err = esp_ota_write(otaHandle, data, dataLen);
  if (err != ESP_OK) {
    sendOtaNack(seq, OTA_ERR_FLASH_ERROR);
    otaAbortCleanup();
    return;
  }
  
  // Update running CRC32
  if (otaHashType == OTA_HASH_CRC32) {
    otaRunningCrc32 = crc32_update(otaRunningCrc32, data, dataLen);
  }
  
  otaBytesWritten += dataLen;
  otaLastChunkTime = millis();
  
  // Send ACK_OTA_CHUNK: bytes_written(4), progress_pct(1)
  uint8_t buf[5];
  memcpy(buf, &otaBytesWritten, 4);
  buf[4] = (uint8_t)((otaBytesWritten * 100) / otaTotalSize);
  sendFrame(seq, RSP_OTA_CHUNK, buf, 5);
}

static void handleOtaEnd(uint16_t seq) {
  // Finalize OTA
  esp_err_t err = esp_ota_end(otaHandle);
  otaHandle = 0;
  
  if (err != ESP_OK) {
    sendOtaNack(seq, OTA_ERR_FLASH_ERROR);
    otaAbortCleanup();
    return;
  }
  
  // Verify checksum
  bool checksumOk = true;
  if (otaHashType == OTA_HASH_CRC32) {
    // Finalize CRC-32: XOR with 0xFFFFFFFF
    uint32_t finalCrc = otaRunningCrc32 ^ 0xFFFFFFFF;
    checksumOk = (finalCrc == otaExpectedCrc32);
  }
  // SHA256 verification would require reading back from flash - skip for now
  
  if (!checksumOk) {
    sendOtaNack(seq, OTA_ERR_CHECKSUM_FAIL);
    otaAbortCleanup();
    return;
  }
  
  // COMMIT: Set boot partition to the new image
  err = esp_ota_set_boot_partition(otaPartition);
  if (err != ESP_OK) {
    sendOtaNack(seq, OTA_ERR_FLASH_ERROR);
    otaAbortCleanup();
    return;
  }
  
  // Send ACK_OTA_DONE: status(1) = 0 (OK)
  uint8_t buf[1] = { 0 };
  sendFrame(seq, RSP_OTA_DONE, buf, 1);
  
  // Clean up state
  otaPartition = nullptr;
  otaTotalSize = 0;
  otaBytesWritten = 0;
  otaInProgress = false;  // Clear OTA in progress flag
  gimbalState = IDLE;
  
  // Reboot after short delay
  delay(500);
  esp_restart();
}

static void handleOtaAbort(uint16_t seq) {
  sendOtaNack(seq, OTA_ERR_ABORTED);
  otaAbortCleanup();
}

// Read version from partition's app description (at offset 0x20 in app image)
// esp_app_desc_t.version is at offset 4 in the struct; struct starts after image header
static void getPartitionVersion(const esp_partition_t* part, char* buf, size_t bufSize) {
  if (part == nullptr || buf == nullptr || bufSize == 0) return;
  memset(buf, 0, bufSize);
  // App description: 0x20 (image header) + 4 (magic) = 0x24, version is 32 bytes
  const size_t APP_DESC_OFFSET = 0x20;
  const size_t VERSION_OFFSET = 4;  // offset of version in esp_app_desc_t
  uint8_t tmp[FW_VERSION_LEN];
  if (esp_partition_read(part, APP_DESC_OFFSET + VERSION_OFFSET, tmp, FW_VERSION_LEN) == ESP_OK) {
    size_t len = 0;
    while (len < bufSize - 1 && tmp[len] >= 0x20 && tmp[len] < 0x7F) len++;
    if (len > 0) {
      memcpy(buf, tmp, len);
    } else {
      memcpy(buf, "---", 3);
    }
  } else {
    memcpy(buf, "---", 3);
  }
}

static void handleGetFwInfo(uint16_t seq) {
  uint8_t payload[1 + FW_VERSION_LEN * 2];
  memset(payload, 0, sizeof(payload));
  
  const esp_partition_t* running = esp_ota_get_running_partition();
  const esp_partition_t* partA = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, nullptr);
  const esp_partition_t* partB = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, nullptr);
  
  uint8_t activeSlot = 0xFF;  // Unknown
  if (running != nullptr) {
    if (partA != nullptr && running->address == partA->address) activeSlot = 0;  // A
    else if (partB != nullptr && running->address == partB->address) activeSlot = 1;  // B
  }
  payload[0] = activeSlot;
  
  char versionA[FW_VERSION_LEN];
  char versionB[FW_VERSION_LEN];
  memset(versionA, 0, FW_VERSION_LEN);
  memset(versionB, 0, FW_VERSION_LEN);
  
  // Use FW_VERSION for active partition (we're running it); read other from flash
  if (activeSlot == 0) {
    size_t len = strlen(FW_VERSION);
    if (len >= FW_VERSION_LEN) len = FW_VERSION_LEN - 1;
    memcpy(versionA, FW_VERSION, len);
    getPartitionVersion(partB, versionB, FW_VERSION_LEN);
  } else if (activeSlot == 1) {
    getPartitionVersion(partA, versionA, FW_VERSION_LEN);
    size_t len = strlen(FW_VERSION);
    if (len >= FW_VERSION_LEN) len = FW_VERSION_LEN - 1;
    memcpy(versionB, FW_VERSION, len);
  } else {
    getPartitionVersion(partA, versionA, FW_VERSION_LEN);
    getPartitionVersion(partB, versionB, FW_VERSION_LEN);
  }
  
  memcpy(&payload[1], versionA, FW_VERSION_LEN);
  memcpy(&payload[1 + FW_VERSION_LEN], versionB, FW_VERSION_LEN);
  
  sendFrame(seq, RSP_FW_INFO, payload, sizeof(payload));
}

static void handleSwitchFw(uint16_t seq, const uint8_t* payload, size_t payloadLen) {
  if (payloadLen < 1) {
    sendNack(seq, 4);
    return;
  }
  uint8_t slot = payload[0];
  if (slot > 1) {
    sendNack(seq, 4);  // Invalid slot
    return;
  }
  
  esp_partition_subtype_t subtype = (slot == 0) ? ESP_PARTITION_SUBTYPE_APP_OTA_0 : ESP_PARTITION_SUBTYPE_APP_OTA_1;
  const esp_partition_t* target = esp_partition_find_first(ESP_PARTITION_TYPE_APP, subtype, nullptr);
  if (target == nullptr) {
    sendNack(seq, 4);  // Partition not found
    return;
  }
  
  esp_err_t err = esp_ota_set_boot_partition(target);
  if (err != ESP_OK) {
    sendNack(seq, 4);
    return;
  }
  
  sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
  delay(500);
  esp_restart();
}

static void processCommand(uint16_t seq, uint16_t type, const uint8_t* payload, size_t payloadLen) {
  // In OTA_RECEIVE state, only accept OTA commands
  if (gimbalState == OTA_RECEIVE && !isOtaType(type)) {
    sendNack(seq, 3);  // state rejected
    return;
  }
  
  // OTA commands only allowed from IDLE state (or within OTA_RECEIVE)
  if (isOtaType(type) && gimbalState != IDLE && gimbalState != OTA_RECEIVE) {
    sendNack(seq, 3);
    return;
  }
  
  if (gimbalState == TRACKING && isConfigType(type)) {
    sendNack(seq, 3);
    return;
  }
  if (gimbalState == CONFIG && isMoveType(type)) {
    sendNack(seq, 3);
    return;
  }

  switch (type) {
    // OTA commands
    case CMD_OTA_START:
      handleOtaStart(seq, payload, payloadLen);
      return;
    case CMD_OTA_CHUNK:
      if (gimbalState != OTA_RECEIVE) { sendNack(seq, 3); return; }
      handleOtaChunk(seq, payload, payloadLen);
      return;
    case CMD_OTA_END:
      if (gimbalState != OTA_RECEIVE) { sendNack(seq, 3); return; }
      handleOtaEnd(seq);
      return;
    case CMD_OTA_ABORT:
      if (gimbalState != OTA_RECEIVE) { sendNack(seq, 3); return; }
      handleOtaAbort(seq);
      return;
    case CMD_GET_FW_INFO:
      if (gimbalState == OTA_RECEIVE) { sendNack(seq, 3); return; }
      handleGetFwInfo(seq);
      return;
    case CMD_SWITCH_FW:
      if (gimbalState == OTA_RECEIVE) { sendNack(seq, 3); return; }
      handleSwitchFw(seq, payload, payloadLen);
      return;
    case CMD_ENTER_TRACKING:
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
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
      // Block commands if safety stop is active
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      float pan, tilt;
      memcpy(&pan, &payload[0], 4); memcpy(&tilt, &payload[4], 4);
      uint16_t spd = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);
      uint16_t acc = (uint16_t)payload[10] | ((uint16_t)payload[11] << 8);
      spd = validateSpeed(spd); acc = validateAccel(acc);  // SAFETY: Validate params
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
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      float pan, tilt;
      memcpy(&pan, &payload[0], 4); memcpy(&tilt, &payload[4], 4);
      uint16_t spdPan = (uint16_t)payload[8] | ((uint16_t)payload[9] << 8);
      uint16_t spdTilt = (uint16_t)payload[10] | ((uint16_t)payload[11] << 8);
      spdPan = validateSpeed(spdPan); spdTilt = validateSpeed(spdTilt);
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
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      float pan; memcpy(&pan, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      uint16_t acc = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
      spd = validateSpeed(spd); acc = validateAccel(acc);
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
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      float tilt; memcpy(&tilt, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      uint16_t acc = (uint16_t)payload[6] | ((uint16_t)payload[7] << 8);
      spd = validateSpeed(spd); acc = validateAccel(acc);
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
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      float pan; memcpy(&pan, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      spd = validateSpeed(spd);
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
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      float tilt; memcpy(&tilt, &payload[0], 4);
      uint16_t spd = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);
      spd = validateSpeed(spd);
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
      if (safetyStopTriggered) { sendNack(seq, SAFETY_ERR_STALL); return; }
      int8_t x = (int8_t)payload[0], y = (int8_t)payload[1];
      uint16_t spd = (uint16_t)payload[2] | ((uint16_t)payload[3] << 8);
      spd = validateSpeed(spd == 0 ? 300 : spd);
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
    case CMD_GET_STATE: {
      uint8_t stateByte = (uint8_t)gimbalState;  // 0=IDLE, 1=TRACKING, 2=CONFIG, 3=OTA_RECEIVE
      sendFrame(seq, FEEDBACK_STATE, &stateByte, 1);
    } break;
    case CMD_RESET_SAFETY: {
      // Reset safety state and re-enable servos
      resetSafetyState();
      ensurePositionMode(PAN_SERVO_ID);
      ensurePositionMode(TILT_SERVO_ID);
      delay(5);
      st.EnableTorque(PAN_SERVO_ID, 1);
      st.EnableTorque(TILT_SERVO_ID, 1);
      panLocked = true;
      tiltLocked = true;
      sendFrame(seq, RSP_ACK_EXECUTED, nullptr, 0);
    } break;
    case CMD_GET_SAFETY_STATUS: {
      // Return safety status: triggered(1), reason(1), panStall(1), tiltStall(1), thermal(1)
      uint8_t buf[5];
      buf[0] = safetyStopTriggered ? 1 : 0;
      buf[1] = safetyStopReason;
      buf[2] = panInStall ? 1 : 0;
      buf[3] = tiltInStall ? 1 : 0;
      buf[4] = thermalThrottleActive ? 1 : 0;
      sendFrame(seq, (uint16_t)1014, buf, 5);  // FEEDBACK_SAFETY_STATUS
    } break;
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
  // SAFETY FIX 1.1: Set servo EPROM angle limits (hardware protection)
  setPanServoAngleLimits();   // Pan: -180° to +180°
  setTiltServoAngleLimits();  // Tilt: -90° to +120°
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
    // SAFETY FIX 5.2: Check bus current immediately after reading
    checkBusCurrent();
  }

  // SAFETY: Run safety checks (stall detection, mode verification)
  if (!safetyStopTriggered && gimbalState != OTA_RECEIVE) {
    runSafetyChecks();
  }
  // Disable periodic feedback reads to avoid interference
  // Only read when explicitly requested (via sendServoFeedback)
  // if (now - lastServoFbMs >= SERVO_FB_MS) {
  //   readServoFeedback(PAN_SERVO_ID, panFb);
  //   readServoFeedback(TILT_SERVO_ID, tiltFb);
  //   lastServoFbMs = now;
  // }

  // OTA timeout handling - only check if OTA is actually in progress (otaHandle is valid)
  // Use fresh millis() to avoid underflow when otaLastChunkTime was just set in this loop iteration
  uint32_t otaNow = millis();
  if (gimbalState == OTA_RECEIVE && otaHandle != 0 && otaNow >= otaLastChunkTime && (otaNow - otaLastChunkTime) > OTA_TIMEOUT_MS) {
    sendOtaNack(0, OTA_ERR_TIMEOUT);
    otaAbortCleanup();
  }
  
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

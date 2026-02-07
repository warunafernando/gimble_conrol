#include "IMU.h"
#include <Arduino.h>

void calibrateMagn();
void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float halfT);
float invSqrt(float x);

#define IMU_DT_MIN 0.005f   // 5 ms min (avoid div-by-zero / numerical issues)
#define IMU_DT_MAX 0.5f     // 500 ms max (long gaps: use default)
#define IMU_DT_DEFAULT 0.02f  // 20 ms default for first call

/******************************************************************************
 * IMU module                                                                 *
 ******************************************************************************/
// #define S_SCL   33
// #define S_SDA   32

AK09918_err_type_t err;

QMI8658 qmi8658_;
AK09918 magnetometer_;

int16_t offset_x = -12, offset_y = 0, offset_z = 0;
int16_t x, y, z;
// Find the magnetic declination at your location
// http://www.magnetic-declination.com/
double declination_shenzhen = -3.22;

#define Kp 2.0f    // proportional gain
#define Ki 1.0f    // integral gain
#define MAG_MIN_SQ 2500.0f  // min mx^2+my^2+mz^2; below this skip mag
#define IMU_USE_MAG 0       // 0=gyro-integrated yaw; 1=AHRS mag fusion
#define YAW_MAG_STATIONARY 0  // 1=use mag when stationary (can jump from interference); 0=gyro only
#define STATIONARY_THRESHOLD_DPS 5.0f  // |yaw rate| < this = stationary, use mag
#define STATIONARY_MAG_BLEND 0.4f      // when stationary: pull toward mag by this fraction per step
#define YAW_MAG_BLEND 0.0f            // when moving: 0=no mag (no jumps); 0.02=slow correction
#define YAW_OFFSET_DEG 0.0f            // add to output (0 = no offset)
#define YAW_GYRO_AXIS_AUTO 1  // 1=detect vertical from acc (use that gyro for yaw); 0=use YAW_GYRO_AXIS
#define YAW_GYRO_AXIS 2     // when AUTO=0: 0=X, 1=Y, 2=Z (axis that is vertical when board horizontal)
#define YAW_GYRO_SIGN 1     // 1 or -1: rotation direction

float angles[3];
float q0, q1, q2, q3;
static uint32_t lastImuMs = 0;
static float yawIntegrated = 0.0f;  // deg, gyro-only when IMU_USE_MAG=0
static uint8_t yawFirstRun = 1;

void imuInit()
{
    // Wire.begin(S_SDA, S_SCL);
    // Serial.begin(115200);
   
    if (qmi8658_.begin() == 0)
	      Serial.println("qmi8658_init fail");

    if (magnetometer_.initialize())
        Serial.println("AK09918_init fail") ;
    magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
    err = magnetometer_.isDataReady();
    int retry_times = 0;
    while (err != AK09918_ERR_OK) {
        Serial.println(err);
        Serial.println("Waiting Sensor");
        delay(100);
        magnetometer_.reset();
        delay(100);
        magnetometer_.switchMode(AK09918_CONTINUOUS_100HZ);
        err = magnetometer_.isDataReady();
        retry_times ++;
        if (retry_times > 10) {
          break;
        }
    }
    // Serial.println("Start figure-8 calibration after 1 seconds.");
    // delay(1000);
    // calibrate(10000, &offset_x, &offset_y, &offset_z);
    // calibrateMagn();
    q0 = 1.0f;  
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}

void imuDataGet(EulerAngles *pstAngles, 
                IMU_ST_SENSOR_DATA_FLOAT *pstGyroRawData,
                IMU_ST_SENSOR_DATA_FLOAT *pstAccelRawData,
                IMU_ST_SENSOR_DATA *pstMagnRawData)
{

  float  acc[3], gyro[3];
  float MotionVal[9];

  magnetometer_.getData(&x, &y, &z);

  pstMagnRawData->s16X = x- offset_x;
  pstMagnRawData->s16Y = y- offset_y;
  pstMagnRawData->s16Z = z- offset_z;

  // qmi8658_.GetEulerAngles(&pstAngles->pitch,&pstAngles->roll,&pstAngles->yaw,acc,gyro);
  qmi8658_.read_sensor_data(acc,gyro);

  // pstAngles->roll = atan2((float)acc[1], (float)acc[2]);
  // pstAngles->pitch = atan2(-(float)acc[0], sqrt((float)(acc[1] * acc[1]) + (float)(acc[2] * acc[2])));

  // double Xheading = pstMagnRawData->s16X * cos(pstAngles->pitch) + pstMagnRawData->s16Y * sin(pstAngles->roll) * sin(pstAngles->pitch) + pstMagnRawData->s16Z * cos(pstAngles->roll) * sin(pstAngles->pitch);
  // double Yheading = pstMagnRawData->s16Y * cos(pstAngles->roll) - pstMagnRawData->s16Z * sin(pstAngles->pitch);
  
  // pstAngles->yaw = /*180 + */57.3 * atan2(Yheading, Xheading) + declination_shenzhen;

  // pstAngles->roll = atan2((float)acc[1], (float)acc[2]) * 57.3;
  // pstAngles->pitch = atan2(-(float)acc[0], sqrt((float)(acc[1] * acc[1]) + (float)(acc[2] * acc[2]))) * 57.3;
  MotionVal[0]=gyro[0];
  MotionVal[1]=gyro[1];
  MotionVal[2]=gyro[2];
  MotionVal[3]=acc[0];
  MotionVal[4]=acc[1];
  MotionVal[5]=acc[2];
  MotionVal[6]=pstMagnRawData->s16X;
  MotionVal[7]=pstMagnRawData->s16Y;
  MotionVal[8]=pstMagnRawData->s16Z;

  // Variable time step: use actual elapsed time for correct gyro integration
  uint32_t now = millis();
  float dt = (lastImuMs == 0) ? IMU_DT_DEFAULT : (float)(now - lastImuMs) * 0.001f;
  if (dt < IMU_DT_MIN) dt = IMU_DT_MIN;
  if (dt > IMU_DT_MAX) dt = IMU_DT_DEFAULT;  // long gap: use default
  lastImuMs = now;
  float halfT = dt * 0.5f;

  imuAHRSupdate((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175, (float)MotionVal[2] * 0.0175,
                (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5], 
                (float)MotionVal[6], (float)MotionVal[7], MotionVal[8], halfT);


  pstAngles->pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; // pitch
  pstAngles->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll

  if (IMU_USE_MAG) {
    pstAngles->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3;
  } else {
    // Gyro-integrated yaw: use the gyro axis that is vertical (rotation in horizontal plane = yaw).
    int yawAxis = YAW_GYRO_AXIS;
    if (YAW_GYRO_AXIS_AUTO) {
      // Board horizontal: acc axis with largest magnitude is vertical; use that gyro for yaw rate.
      float ax = acc[0], ay = acc[1], az = acc[2];
      float ax2 = ax*ax, ay2 = ay*ay, az2 = az*az;
      if (az2 >= ax2 && az2 >= ay2) yawAxis = 2;
      else if (ay2 >= ax2 && ay2 >= az2) yawAxis = 1;
      else yawAxis = 0;
    }
    float yawRateDeg = gyro[yawAxis];  // deg/s
    float dtSec = dt * 1.0f;
    yawIntegrated += (float)YAW_GYRO_SIGN * yawRateDeg * dtSec;

    int useMag = 0;
    float magBlend = 0.0f;
    if (YAW_MAG_STATIONARY) {
      float yawRateAbs = (float)fabsf((float)YAW_GYRO_SIGN * yawRateDeg);
      if (yawRateAbs < STATIONARY_THRESHOLD_DPS) {
        useMag = 1;
        magBlend = STATIONARY_MAG_BLEND;  // stationary: use mag for true North
      }
    } else if (YAW_MAG_BLEND > 0.0f) {
      useMag = 1;
      magBlend = YAW_MAG_BLEND;
    }

    if (useMag && magBlend > 0.0f) {
      float magSqLoc = (float)pstMagnRawData->s16X * pstMagnRawData->s16X + (float)pstMagnRawData->s16Y * pstMagnRawData->s16Y + (float)pstMagnRawData->s16Z * pstMagnRawData->s16Z;
      if (magSqLoc > MAG_MIN_SQ) {
        float rollRad = pstAngles->roll * 0.01745f, pitchRad = pstAngles->pitch * 0.01745f;
        float mxf = (float)pstMagnRawData->s16X, myf = (float)pstMagnRawData->s16Y, mzf = (float)pstMagnRawData->s16Z;
        float Xh = mxf * cosf(pitchRad) + myf * sinf(rollRad) * sinf(pitchRad) + mzf * cosf(rollRad) * sinf(pitchRad);
        float Yh = myf * cosf(rollRad) - mzf * sinf(rollRad);
        float yawMag = atan2f(Yh, Xh) * 57.3f + (float)declination_shenzhen;
        if (yawFirstRun) {
          yawIntegrated = yawMag;
          yawFirstRun = 0;
        } else {
          float diff = yawMag - yawIntegrated;
          while (diff > 180.0f) diff -= 360.0f;
          while (diff < -180.0f) diff += 360.0f;
          yawIntegrated += magBlend * diff;
        }
      }
    } else {
      yawFirstRun = 0;
    }
    while (yawIntegrated > 180.0f) yawIntegrated -= 360.0f;
    while (yawIntegrated < -180.0f) yawIntegrated += 360.0f;
    pstAngles->yaw = yawIntegrated + YAW_OFFSET_DEG;
    if (pstAngles->yaw > 180.0f) pstAngles->yaw -= 360.0f;
    if (pstAngles->yaw < -180.0f) pstAngles->yaw += 360.0f;
  }

  pstGyroRawData->X = gyro[0];
  pstGyroRawData->Y = gyro[1];
  pstGyroRawData->Z = gyro[2];

  pstAccelRawData->X = acc[0];
  pstAccelRawData->Y = acc[1];
  pstAccelRawData->Z = acc[2];

  return;  
}

void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float halfT) 
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
  float ex, ey, ez;

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;   
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;          

  norm = invSqrt(ax * ax + ay * ay + az * az);       
  ax = ax * norm;
  ay = ay * norm;
  az = az * norm;

  float magSq = mx * mx + my * my + mz * mz;
  int useMag = IMU_USE_MAG && (magSq > MAG_MIN_SQ) ? 1 : 0;  // skip mag if disabled or too weak
  if (useMag) {
    norm = invSqrt(magSq);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;
  }

  // compute reference direction of flux (only if mag valid)
  if (useMag) {
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);         
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;     

    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);  

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  } else {
    // accel-only: no mag correction (yaw from gyro only this step)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);
  }

  if(ex != 0.0f || ey != 0.0f || ez != 0.0f)
  {
    exInt = exInt + ex * Ki * halfT;
    eyInt = eyInt + ey * Ki * halfT;  
    ezInt = ezInt + ez * Ki * halfT;

    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
  }

  q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
  q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
  q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
  q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;  

  norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 = q0 * norm;
  q1 = q1 * norm;
  q2 = q2 * norm;
  q3 = q3 * norm;

  // ensure q0 >= 0 (q and -q are same rotation; prevents 180° yaw jumps)
  if (q0 < 0.0f) {
    q0 = -q0; q1 = -q1; q2 = -q2; q3 = -q3;
  }
}

float invSqrt(float x) 
{
  float halfx = 0.5f * x;
  float y = x;
  
  long i = *(long*)&y;                //get bits for floating value
  i = 0x5f3759df - (i >> 1);          //gives initial guss you
  y = *(float*)&i;                    //convert bits back to float
  y = y * (1.5f - (halfx * y * y));   //newtop step, repeating increases accuracy
  
  return y;
}

void calibrateMagn(void)
{
  int16_t temp[9];
  Serial.printf("keep 10dof-imu device horizontal and it will read x y z axis offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[0] = x;
  temp[1] = y;
  temp[2] = z;
  
  Serial.printf("rotate z axis 180 degrees and it will read all axises offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[3] = x;
  temp[4] = y;
  temp[5] = z;

  Serial.printf("flip 10dof-imu device and keep it horizontal and it will read all axises offset value after 4 seconds\n");
  delay(4000);
  Serial.printf("start read all axises offset value\n");
  magnetometer_.getData(&x, &y, &z);
  temp[6] = x;
  temp[7] = y;
  temp[8] = z;

  offset_x = (temp[0]+temp[3])/2;
  offset_y = (temp[1]+temp[4])/2;
  offset_z = (temp[5]+temp[8])/2;
}

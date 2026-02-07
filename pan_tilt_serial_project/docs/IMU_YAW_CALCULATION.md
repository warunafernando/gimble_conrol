# How IMU Yaw (Y) Is Calculated on ESP32

This document describes how the **yaw** value in the IMU response (type 1002) is computed in the firmware. It is the value driving the "Heading (Yaw)" gauge in the GUI.

---

## Data flow

1. **`sendImuBinary(seq)`** (pan_tilt_serial_project.ino)  
   Calls `updateImu()` then copies `imuAngles.yaw` (float) into the binary payload at bytes 8–11 (after roll, pitch).

2. **`updateImu()`**  
   Calls `imuDataGet(&imuAngles, &imuGyro, &imuAccel, &imuMagn)`.

3. **`imuDataGet()`** (IMU.cpp)  
   Reads sensors, runs the AHRS fusion, then derives Euler angles from the quaternion. **Yaw is set here.**

---

## Yaw formula (IMU.cpp)

Yaw is taken from the quaternion `(q0, q1, q2, q3)`:

```c
pstAngles->yaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.3;
```

- `57.3` ≈ 180/π converts radians to degrees.
- This is the standard formula for **yaw** from a quaternion (Z-axis rotation, yaw-pitch-roll convention).

So **yaw is not computed from the magnetometer directly**; it comes from the **fused quaternion** produced by `imuAHRSupdate()`.

---

## Where the quaternion comes from: `imuAHRSupdate()`

The quaternion is updated in **IMU.cpp** in `imuAHRSupdate()` with:

- **Gyro** (rad/s): from QMI8658, scaled by `0.0175` (deg/s → rad/s) in `imuDataGet()`:
  ```c
  imuAHRSupdate((float)MotionVal[0] * 0.0175, (float)MotionVal[1] * 0.0175, (float)MotionVal[2] * 0.0175, ...);
  ```
- **Accel** (normalized): from QMI8658, normalized inside `imuAHRSupdate()`.
- **Magnetometer** (normalized): from AK09918, raw `(x - offset_x, y - offset_y, z - offset_z)` then normalized in `imuAHRSupdate()`.

So **yaw is fused from gyro + accel + magnetometer** (Madgwick-style correction). The magnetometer is what keeps yaw from drifting; without it, yaw would be gyro-only and drift.

---

## Potential causes of buggy yaw

### 1. Fixed time step `halfT = 0.024f`

In **IMU.cpp** line 130:

```c
float ex, ey, ez, halfT = 0.024f; /*half the sample period*/
```

- `halfT` is **fixed at 0.024 s** (48 ms period).
- The real interval between `imuDataGet()` calls varies (e.g. 100 ms when using feedback interval 100 ms, or on demand).
- **Effect:** Gyro integration uses the wrong dt. If the actual period is 100 ms, the quaternion (and thus yaw) will integrate too slowly and lag or be inconsistent. If the period is faster than 48 ms, integration will overcorrect.

**Recommendation:** Measure the actual time between calls and pass it into `imuAHRSupdate()`, and use that for the quaternion update and integral terms (replace the fixed `halfT`).

### 2. Magnetometer only partially offset

In **IMU.cpp**:

```c
int16_t offset_x = -12, offset_y = 0, offset_z = 0;
// ...
pstMagnRawData->s16X = x - offset_x;
pstMagnRawData->s16Y = y - offset_y;
pstMagnRawData->s16Z = z - offset_z;
```

- Only **X** has a non-zero offset; Y and Z are 0.
- If the mag has bias on Y or Z, the fused yaw will be biased or noisy.

**Recommendation:** Run the existing `calibrateMagn()` (or equivalent) and set `offset_x`, `offset_y`, `offset_z` from calibration.

### 3. Magnetic declination not applied

Commented code in **IMU.cpp** (lines 87–90) shows a direct mag-based yaw with declination:

```c
// pstAngles->yaw = /*180 + */57.3 * atan2(Yheading, Xheading) + declination_shenzhen;
```

- `declination_shenzhen = -3.22` is defined but **never used** in the current AHRS path.
- The AHRS yaw is in sensor frame; for “North = 0°” you’d add local magnetic declination. So the number is not wrong, but it’s not a true geographic heading unless you add declination (e.g. in firmware or in the GUI).

### 4. Magnetometer sensitivity to environment

- The AHRS uses the **direction** of the mag field (normalized). Nearby motors, cables, or metal can distort the field and make yaw jump or drift.
- If the gimbal is close to the ESP32/servos, magnetic interference can make yaw look “buggy.”

### 5. Quaternion convention / gimbal lock

- The Euler extraction (pitch, roll, yaw) from the quaternion can be singular near pitch ±90°. If the gimbal tilts a lot, yaw can flip or behave oddly near that region.

---

## Summary table

| Item              | Location        | Detail |
|-------------------|-----------------|--------|
| Yaw output        | IMU.cpp:111     | `atan2(-2*q1*q2 - 2*q0*q3, 2*q2*q2 + 2*q3*q3 - 1) * 57.3` (degrees) |
| Quaternion update | IMU.cpp:124–191 | `imuAHRSupdate()`: gyro (rad/s) + accel + mag, fixed `halfT = 0.024f` |
| Gyro scaling      | IMU.cpp:104     | `* 0.0175` (deg/s → rad/s) |
| Mag offset        | IMU.cpp:19,77–80| Only `offset_x = -12` used; Y/Z = 0 |
| Declination       | IMU.cpp:23,90   | Defined but not applied to current yaw |
| Call rate         | ino             | On demand (cmd 126) or every feedback interval (e.g. 100 ms) |

The most likely fix for “buggy” yaw is to **use a correct, variable time step** in the AHRS (instead of `halfT = 0.024f`) and to **calibrate magnetometer offsets** for all axes.

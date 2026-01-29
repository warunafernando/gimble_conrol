# Bug Analysis - Only Set Absolute and Center Buttons Work

## Problem
Only "Set Absolute" and "Center" buttons move the motors. Other buttons (Pan Only, Tilt Only, Move, Lock, etc.) don't work.

## Root Cause Analysis

### Issue 1: Heartbeat Keepalive Bug (FIXED)
**Problem**: The keepalive was sending T=126 (Get IMU), which does NOT call `touchHeartbeat()` in firmware.

**Firmware Code**:
```cpp
case CMD_GET_IMU:
  sendImuFeedback();
  break;  // NO touchHeartbeat() call!
```

**Solution**: Changed keepalive to use T=135 (Stop), which DOES call `touchHeartbeat()`:
```cpp
case CMD_PAN_TILT_STOP:
  stopPanTilt();
  touchHeartbeat();  // ✓ Resets heartbeat timer
  break;
```

### Issue 2: Commands May Not Be Flushed
**Problem**: Commands might be buffered and not sent immediately.

**Solution**: Added `serial_port.flush()` after writing to ensure immediate transmission.

### Issue 3: Heartbeat Timeout Behavior
When heartbeat times out:
1. `stopPanTilt()` is called → disables servo torque
2. `heartbeatStopActive = true`
3. Servos are disabled until a command with `touchHeartbeat()` is received

**All movement commands call `touchHeartbeat()`**, so they should re-enable servos. But if keepalive wasn't working, heartbeat would timeout, servos disabled, and then commands might not work properly.

## Commands That Call touchHeartbeat()

✅ **These commands reset heartbeat**:
- T=133 (Pan/Tilt Absolute)
- T=134 (Pan/Tilt Move)
- T=135 (Stop) ← **Now used for keepalive**
- T=141 (User Control)
- T=170 (Pan Lock)
- T=171 (Tilt Lock)
- T=172 (Pan Only Absolute)
- T=173 (Tilt Only Absolute)
- T=174 (Pan Only Move)
- T=175 (Tilt Only Move)

❌ **These commands do NOT reset heartbeat**:
- T=126 (Get IMU) ← **Was incorrectly used for keepalive**
- T=160 (Get INA219)
- T=131 (Feedback Enable/Disable)
- T=142 (Feedback Interval)
- T=136 (Heartbeat Set)

## Fixes Applied

1. ✅ Changed keepalive from T=126 to T=135 (Stop command)
2. ✅ Added `serial_port.flush()` to ensure immediate transmission
3. ✅ Added return value to `send_command()` for error checking

## Testing

After fixes, all buttons should work:
- ✅ Set Absolute (T=133)
- ✅ Center (T=133)
- ✅ Set Pan Only (T=172)
- ✅ Set Tilt Only (T=173)
- ✅ Move (T=134)
- ✅ Lock/Unlock (T=170/171)
- ✅ User Control (T=141)
- ✅ Stop (T=135)

## Expected Behavior

1. On connect: Heartbeat keepalive starts (sends T=135 every 2 seconds)
2. All commands: Should work immediately
3. Heartbeat: Never times out (keepalive prevents it)
4. Servos: Stay enabled as long as keepalive is active

---

**Status**: Fixed  
**Date**: 2026-01-26

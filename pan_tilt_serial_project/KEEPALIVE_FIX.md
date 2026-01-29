# Keepalive Fix - Servos Being Disabled

## Problem
The keepalive was sending T=135 (Stop) every 2 seconds, which **disables servo torque**. This caused servos to be disabled right when users tried to move them.

## Root Cause
- T=135 (Stop) calls `touchHeartbeat()` ✓ (resets heartbeat)
- BUT it also calls `stopPanTilt()` which does:
  ```cpp
  st.EnableTorque(PAN_SERVO_ID, 0);  // ❌ Disables servos!
  st.EnableTorque(TILT_SERVO_ID, 0);
  ```

## Solution
Changed keepalive to use T=170/171 (Lock) commands:
- T=170 (Pan Lock) calls `touchHeartbeat()` ✓
- T=171 (Tilt Lock) calls `touchHeartbeat()` ✓
- Both enable torque and hold position (servos stay enabled) ✓
- Alternates between pan and tilt every 2 seconds

## Additional Fix
- Increased default heartbeat timeout to 30 seconds (from 3 seconds)
- This makes keepalive less critical - even if it fails, there's a 30-second window

## How Lock Commands Work
```cpp
case CMD_PAN_LOCK: {
  int cmdVal = jsonCmdReceive["cmd"] | 0;
  if (cmdVal == 1) {
    lockAxis(PAN_SERVO_ID, panFb, panLocked);  // Enables torque, holds position
  }
  touchHeartbeat();  // ✓ Always called, even if already locked
} break;
```

`lockAxis()` enables torque and holds current position - perfect for keepalive!

---

**Status**: Fixed  
**Date**: 2026-01-26

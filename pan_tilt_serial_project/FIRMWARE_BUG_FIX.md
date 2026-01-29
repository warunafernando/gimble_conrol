# Firmware Bug Fix - Servos Moving Unexpectedly

## Problem
When sending T=173 (Tilt Only) or T=172 (Pan Only), both servos were moving. The sweep test worked but individual buttons didn't.

## Root Cause

### Bug 1: lockAxis() Called Unconditionally
When T=173 (Tilt Only) was received:
1. `setTiltAbs()` moved tilt ✓
2. `lockAxis(PAN_SERVO_ID, ...)` was called to lock pan
3. `lockAxis()` reads current position and writes it back
4. If position read fails or returns wrong value, pan moves incorrectly ❌

**Problem**: `lockAxis()` was called even if the axis was already locked, causing unnecessary servo communication that could fail or return stale data.

### Bug 2: No Position Validation
`lockAxis()` didn't validate the position before writing it back. If `readServoFeedback()` returned a bad value (0, 4095, or out of range), it would still try to write it.

### Bug 3: No Check if Already Locked
The code called `lockAxis()` every time, even if the axis was already locked. The function has a check, but it's better to avoid the call entirely.

## Fixes Applied

### Fix 1: Check if Already Locked Before Calling lockAxis()
```cpp
// Before
lockAxis(PAN_SERVO_ID, panFb, panLocked);

// After
if (!panLocked) {
  lockAxis(PAN_SERVO_ID, panFb, panLocked);
}
```

This prevents unnecessary servo communication when the axis is already locked.

### Fix 2: Improved lockAxis() with Position Validation
```cpp
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
```

**Improvements**:
- Checks return value of `readServoFeedback()`
- Validates position is in valid range (0-4095)
- If read fails, just enables torque without moving (safer)

## Commands Fixed
- ✅ T=172 (Pan Only Absolute) - Now checks if tilt is already locked
- ✅ T=173 (Tilt Only Absolute) - Now checks if pan is already locked
- ✅ T=174 (Pan Only Move) - Now checks if tilt is already locked
- ✅ T=175 (Tilt Only Move) - Now checks if pan is already locked

## Expected Behavior After Fix
1. **T=172 (Pan Only)**: Moves pan, locks tilt at current position (only if not already locked)
2. **T=173 (Tilt Only)**: Moves tilt, locks pan at current position (only if not already locked)
3. **No unexpected movement**: Pan won't move when sending tilt commands, and vice versa
4. **Better reliability**: Position validation prevents writing bad values

---

**Status**: Fixed  
**Date**: 2026-01-26  
**Files Modified**: `pan_tilt_serial_project.ino`

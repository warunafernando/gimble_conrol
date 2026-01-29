# Servo ID Testing

## Current Configuration
- PAN_SERVO_ID = 2
- TILT_SERVO_ID = 1

## Issue
- Pan commands do nothing (suggests ID 2 might be wrong or not connected)
- Tilt commands move both servos (suggests IDs might be swapped or hardware issue)

## Possible Solutions

### Option 1: Try Swapped IDs
If servo IDs are swapped in hardware:
- PAN_SERVO_ID = 1 (currently tilt)
- TILT_SERVO_ID = 2 (currently pan)

### Option 2: Test Individual Servos
Create a test command to move each servo ID individually to verify which is which.

### Option 3: Hardware Check
- Verify servo IDs are actually 1 and 2
- Check servo bus connections
- Verify both servos are powered and responding

---

**Next Step**: Try swapping the IDs in firmware to test if that's the issue.

# Servo Control Issue Analysis

## Observed Behavior

### Original Configuration (PAN=2, TILT=1)
- **Sweep Pan (ID 2)**: Does nothing
- **Sweep Tilt (ID 1)**: Moves both motors

### Swapped Configuration (PAN=1, TILT=2)  
- **Sweep Pan (ID 1)**: Moves both motors
- **Sweep Tilt (ID 2)**: Does nothing

## Root Cause Analysis

### Pattern Identified
- **ID 1**: Works but affects BOTH servos
- **ID 2**: Does NOT respond at all

### Possible Causes

1. **Both servos configured to ID 1** (Most Likely)
   - Both servos are set to the same ID (1)
   - Commands to ID 1 affect both servos
   - Commands to ID 2 have no effect (no servo with that ID)

2. **Hardware wiring issue**
   - Servo bus wired incorrectly
   - Both servos receiving same signal
   - One servo not connected properly

3. **Servo ID configuration**
   - Servos need to be reprogrammed with different IDs
   - One servo might be set to broadcast (0xFE = 254)

## Evidence

- `WritePosEx(ID, ...)` sends to specific ID
- `SyncWritePosEx()` sends to multiple IDs in array
- When ID 1 is used, both servos move
- When ID 2 is used, nothing happens
- This suggests both servos are responding to ID 1

## Solution

### Option 1: Verify Servo IDs
Use the ping command (T=200) to test which IDs respond:
```json
{"T":200,"id":1}  // Test ID 1
{"T":200,"id":2}  // Test ID 2
```

### Option 2: Reprogram Servo IDs
If both are ID 1, one needs to be changed to ID 2:
- Use servo configuration tool
- Or use firmware command to set servo ID (if available)

### Option 3: Use SyncWritePosEx for Both
Since both servos seem to be ID 1, we could:
- Always use `SyncWritePosEx()` with both IDs in array
- But this won't work if we need independent control

## Current Firmware Changes

1. ✅ Disabled periodic feedback reads (reduces interference)
2. ✅ Added delays between servo commands (prevents bus conflicts)
3. ✅ Added ping diagnostic command (T=200)
4. ✅ Swapped servo IDs to test (PAN=1, TILT=2)

## Next Steps

1. **Close GUI** and upload new firmware
2. **Test ping command** to verify which servo IDs respond
3. **If both are ID 1**: Reprogram one servo to ID 2
4. **If hardware issue**: Check servo bus wiring

---

**Status**: Analysis complete, firmware ready for testing  
**Date**: 2026-01-26

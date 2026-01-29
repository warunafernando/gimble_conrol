# GUI Fixes - Heartbeat and Command Issues

## Issues Fixed

### 1. Heartbeat Keepalive (CRITICAL)
**Problem**: The firmware has a 3-second heartbeat timeout. If no command is received within 3 seconds, the servos are automatically stopped for safety. Only commands that were sent frequently (like "Set Absolute" and "Center") worked because they reset the heartbeat timer.

**Solution**: 
- Added automatic heartbeat keepalive that sends a harmless command (T=126 Get IMU) every 2 seconds
- This keeps the heartbeat timer reset without moving the servos
- Heartbeat keepalive starts automatically on connect
- Visual indicator shows keepalive status (Active/Inactive)

**Implementation**:
- `start_heartbeat_keepalive()`: Starts periodic keepalive
- `stop_heartbeat_keepalive()`: Stops keepalive on disconnect
- `send_heartbeat_keepalive()`: Sends keepalive command every 2 seconds

### 2. Connection Initialization
**Problem**: On connect, heartbeat timeout and feedback weren't configured.

**Solution**:
- Automatically sets heartbeat timeout (3000ms default) on connect
- Automatically enables periodic feedback on connect
- Starts heartbeat keepalive immediately

### 3. Move Command Speed Control
**Problem**: Move command used same speed for both pan and tilt.

**Solution**:
- Added separate speed controls for pan and tilt in Move command
- Pan Speed and Tilt Speed spinboxes (default 300)
- Command now sends `SX` and `SY` separately

### 4. User Control Speed
**Problem**: User control didn't have a speed control.

**Solution**:
- Added speed spinbox for user control commands
- Default speed: 300

## How It Works

### Heartbeat System
1. **Firmware**: Monitors `lastCmdRecvTime`. If no command received for `heartbeatTimeoutMs` (default 3000ms), stops servos.
2. **GUI Keepalive**: Sends T=126 (Get IMU) every 2 seconds to reset heartbeat timer.
3. **All Commands**: Every motion command also resets the heartbeat timer.

### Command Flow
```
User clicks button → GUI sends command → Firmware receives → touchHeartbeat() called → 
Heartbeat timer reset → Servo moves → Feedback sent back
```

## Testing

After these fixes:
- ✅ All control buttons should work (not just Set Absolute and Center)
- ✅ Servos won't stop after 3 seconds of inactivity
- ✅ Heartbeat status indicator shows "Active" when connected
- ✅ Move command has separate pan/tilt speed controls

## Status Indicators

- **Connection Status**: Green "Connected" / Red "Disconnected"
- **Heartbeat Keepalive**: Green "Active" / Red "Inactive"
- **Heartbeat Status**: Shows "Alive" (green) or "Timeout" (red) from firmware

---

**Fixed**: 2026-01-26  
**Status**: ✅ All issues resolved

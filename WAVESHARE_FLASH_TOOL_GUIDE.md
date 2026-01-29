# Waveshare ESP32 Flash Download Tool - Official Procedure

Based on: https://www.waveshare.com/wiki/2-Axis_Pan-Tilt_Camera_Module#Product_Firmware_Update

## Key Information

**Waveshare provides a specific ESP32 download tool** for the pan-tilt camera module that includes pre-configured firmware files. This is different from the generic Espressif tool.

## Download the Tool

**Direct Download Link:**
https://files.waveshare.com/wiki/2-Axis-Pan-Tilt-Camera-Module/2-Axis_Pan-Tilt_Camera_Module_FACTORY.zip

**Also available in Open-source Demo section:**
https://www.waveshare.com/wiki/2-Axis_Pan-Tilt_Camera_Module#Open-source_Demo

## Step-by-Step Procedure

### Step 1: Connect Hardware
1. Use a USB cable to connect the **middle-side USB port** of General Driver for Robots to your computer
   - ⚠️ **Important**: Use the USB port labeled "USB" (port #9), NOT the LIDAR Type-C port
2. The board should appear as a COM port in Device Manager

### Step 2: Download and Extract Tool
1. Download: `2-Axis_Pan-Tilt_Camera_Module_FACTORY.zip`
2. Extract the ZIP file
3. Double-click **`flash_download_tool_3.9.5.exe`** to open the program
4. **Two windows will pop up:**
   - UI interface (the one you need to operate)
   - Terminal window (displays working status)

### Step 3: Configure Tool Settings

**In the "DOWNLOAD TOOL MODE" interface:**

1. **Chip Type**: Select **"ESP32"**
2. **WorkMode**: Select **"Factory"** ⚠️ **IMPORTANT!**
   - Factory mode uses **relative paths** for binary files
   - You **do NOT need to manually enter binary file paths**
   - The tool comes with pre-configured firmware files
3. Click **OK** after selecting

### Step 4: Configure Upload Settings

**In the main software interface:**

1. **Keep "LockSettings" checked** ✅
   - The right side shows you can upload to 8 pan-tilts simultaneously
2. **Click "COM"** and select your COM port (e.g., **COM9**)
3. **BAUD**: Set download speed
   - Higher value = faster speed
   - ESP32 can use up to **921600** baud
   - Recommended: Start with **115200**, then try **921600** if it works

### Step 5: Flash Firmware

1. **Put ESP32 in download mode** (if needed):
   - Hold **BOOT/DOWNLOAD** button (button #6)
   - Press and release **RESET** button (button #5)
   - Release **BOOT** button
   - Note: The board has an "Automatic download circuit" (#27), so this may not be necessary

2. **Click "START"** to upload the program

3. **Wait for completion:**
   - Status will change from **"IDLE"** to **"FINISH"**
   - Check the terminal window for progress and any errors

### Step 6: Complete

1. After upload completes ("FINISH" status):
   - Disconnect USB connection between driver board and computer
   - Use the matching **12V 5A power supply** to power the product
   - Turn on the switch on the driver board
   - The robot can be controlled after power is turned on

## Why This Tool Works Better

1. **Pre-configured firmware**: Factory mode includes the correct firmware files with proper offsets
2. **Relative paths**: No need to manually specify file paths
3. **Waveshare-specific**: Designed specifically for this hardware
4. **Automatic download circuit**: The board (#27) may handle download mode automatically

## Important Notes

- **WorkMode MUST be "Factory"** - not "Develop"
- Use the **middle USB port** (labeled "USB"), not the LIDAR port
- The tool includes firmware files - you don't need to provide your own compiled firmware
- If flashing your own custom firmware, you may need to use "Develop" mode instead

## Troubleshooting

- **COM port not showing**: Install CP2102 driver from https://files.waveshare.com/upload/d/d6/CP210x_USB_TO_UART.zip
- **Connection fails**: Make sure you're using the correct USB port (middle-side, labeled "USB")
- **Upload fails**: Try lower baud rate (115200), or manually put ESP32 in download mode
- **Tool won't start**: Make sure you extracted all files from the ZIP

## For Custom Firmware

If you want to flash your own compiled firmware (like from PlatformIO), you may need to:
1. Use "Develop" mode instead of "Factory"
2. Manually configure file paths and addresses:
   - Bootloader: `0x1000`
   - Partitions: `0x8000`
   - Boot App0: `0xe000`
   - Application: `0x10000`

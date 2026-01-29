# CP2102 Driver Installation Guide

## Official Driver Download

The latest CP210x USB to UART Bridge VCP drivers are available from Silicon Labs:

**Official Download Page:**
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers

## Installation Steps

1. **Download the Driver:**
   - Go to: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
   - Download the Windows driver package (usually named something like `CP210x_Universal_Windows_Driver.zip`)

2. **Extract and Install:**
   - Extract the ZIP file to a temporary folder
   - Run the installer:
     - **64-bit Windows:** Run `CP210xVCPInstaller_x64.exe`
     - **32-bit Windows:** Run `CP210xVCPInstaller_x86.exe`
   - Follow the installation wizard

3. **Verify Installation:**
   - Open Device Manager (Win+X → Device Manager)
   - Look under "Ports (COM & LPT)"
   - You should see "Silicon Labs CP210x USB to UART Bridge (COMx)"
   - If you see a yellow warning icon, right-click → Update driver → Browse → Select the extracted driver folder

4. **Unplug and Replug:**
   - Unplug your ESP32 board
   - Wait 5 seconds
   - Plug it back in
   - Check Device Manager again to confirm the COM port appears

## Alternative: Direct Download Link

If the website requires registration, you can also try:
- Waveshare's driver package (if available): https://files.waveshare.com/upload/d/d6/CP210x_USB_TO_UART.zip

## Troubleshooting

If DTR/RTS still don't work after installing the driver:
- Some CP2102 boards don't have DTR/RTS properly connected
- You may need to manually enter download mode (hold BOOT, press RESET)
- Try a different USB cable or USB port
- Check if Windows Update has a newer driver version

## Driver Version Check

To check your current driver version:
1. Device Manager → Ports → Silicon Labs CP210x → Properties
2. Driver tab → Check driver version
3. Compare with latest version on Silicon Labs website

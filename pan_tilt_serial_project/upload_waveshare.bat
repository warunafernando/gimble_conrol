@echo off
REM Upload firmware using Waveshare Flash Download Tool
REM This script uses the Waveshare tool which works reliably with the auto-download circuit

set PROJECT_DIR=%~dp0
set BUILD_DIR=%PROJECT_DIR%.pio\build\esp32dev
set WAVESHARE_TOOL_DIR=%PROJECT_DIR%..\..\Waveshare_Flash_Tool

echo ========================================
echo Waveshare Flash Tool Upload Helper
echo ========================================
echo.
echo This script helps upload firmware using the Waveshare Flash Download Tool
echo which works reliably with the General Driver for Robots board.
echo.
echo Instructions:
echo 1. Make sure the Waveshare Flash Download Tool is installed
echo 2. Put ESP32 in download mode (hold BOOT, press RESET, release BOOT)
echo 3. Run this script
echo.
echo Firmware location: %BUILD_DIR%\firmware.bin
echo.

if not exist "%BUILD_DIR%\firmware.bin" (
    echo ERROR: Firmware not found! Please build the project first.
    echo Run: pio run
    pause
    exit /b 1
)

echo Firmware found: %BUILD_DIR%\firmware.bin
echo.
echo To upload:
echo 1. Open Waveshare Flash Download Tool
echo 2. Select "Develop" mode (not Factory)
echo 3. Configure:
echo    - Bootloader: %BUILD_DIR%\bootloader.bin at 0x1000
echo    - Partitions: %BUILD_DIR%\partitions.bin at 0x8000  
echo    - Application: %BUILD_DIR%\firmware.bin at 0x10000
echo 4. Select COM port (usually COM9)
echo 5. Click START
echo.
echo Opening build directory...
explorer "%BUILD_DIR%"
pause

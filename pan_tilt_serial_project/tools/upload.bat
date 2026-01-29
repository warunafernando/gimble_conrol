@echo off
REM Convenience script to upload firmware using the Waveshare-compatible uploader
REM Automatically detects PlatformIO build directory

set SCRIPT_DIR=%~dp0
set PROJECT_DIR=%SCRIPT_DIR%..
set BUILD_DIR=%PROJECT_DIR%\.pio\build\esp32dev

REM Default COM port (change if needed)
set COM_PORT=COM9

echo ========================================
echo Waveshare-Compatible ESP32 Uploader
echo ========================================
echo.

REM Check if build directory exists
if not exist "%BUILD_DIR%" (
    echo ERROR: Build directory not found!
    echo Please build the project first: pio run
    pause
    exit /b 1
)

REM Check if firmware exists
if not exist "%BUILD_DIR%\firmware.bin" (
    echo ERROR: Firmware not found!
    echo Please build the project first: pio run
    pause
    exit /b 1
)

echo Build directory: %BUILD_DIR%
echo COM port: %COM_PORT%
echo.

REM Run the uploader
python "%SCRIPT_DIR%waveshare_uploader.py" -p %COM_PORT% --build-dir "%BUILD_DIR%"

if %ERRORLEVEL% EQU 0 (
    echo.
    echo Upload completed successfully!
) else (
    echo.
    echo Upload failed!
)

pause

@echo off
REM OTA firmware load - double-click to run, no timeout
REM Ensure nothing else is using COM9 (close GUI, serial monitor, etc.)

setlocal
set PORT=COM9
if not "%~1"=="" set PORT=%~1

cd /d "%~dp0.."

echo ============================================================
echo   UART OTA Firmware Upload
echo   Port: %PORT%
echo ============================================================
echo.

if not exist ".pio\build\esp32dev\firmware.bin" (
    echo Building firmware first...
    pio run -e esp32dev
    if errorlevel 1 exit /b 1
    echo.
)

python tools\uart_ota.py -p %PORT%

echo.
pause

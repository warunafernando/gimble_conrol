@echo off
REM UART OTA Firmware Upload Tool
REM Usage: uart_ota.bat [COM_PORT]
REM Default: COM9

setlocal

set PORT=%1
if "%PORT%"=="" set PORT=COM9

set SCRIPT_DIR=%~dp0
set PROJECT_DIR=%SCRIPT_DIR%..

echo ============================================================
echo   UART OTA Firmware Upload
echo   Port: %PORT%
echo ============================================================
echo.

REM Check if firmware exists
if exist "%PROJECT_DIR%\.pio\build\esp32s3_ota\firmware.bin" (
    set FW_PATH=%PROJECT_DIR%\.pio\build\esp32s3_ota\firmware.bin
    echo Using ESP32-S3 OTA build
) else if exist "%PROJECT_DIR%\.pio\build\esp32dev\firmware.bin" (
    set FW_PATH=%PROJECT_DIR%\.pio\build\esp32dev\firmware.bin
    echo Using ESP32 dev build
) else (
    echo Error: No firmware found. Run 'pio run' first.
    exit /b 1
)

echo Firmware: %FW_PATH%
echo.

REM Run OTA upload
python "%SCRIPT_DIR%uart_ota.py" -p %PORT% -f "%FW_PATH%" -v

if %ERRORLEVEL% EQU 0 (
    echo.
    echo OTA upload successful!
) else (
    echo.
    echo OTA upload failed. You may need to use manual bootloader upload.
    echo Run: load_new_fw.bat
)

endlocal

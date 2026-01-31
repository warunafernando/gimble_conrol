@echo off
REM Load new firmware onto ESP32
REM 
REM This script tries two methods:
REM   1. UART OTA - uploads via running firmware (no bootloader needed)
REM   2. Manual bootloader - fallback if OTA not supported or fails
REM
REM Usage: load_new_fw.bat [COM_PORT] [--manual]
REM   --manual : Skip OTA, go directly to manual bootloader upload

setlocal
set COM_PORT=COM9
set SKIP_OTA=0

:parse_args
if "%~1"=="" goto :args_done
if /I "%~1"=="--manual" (
    set SKIP_OTA=1
    shift
    goto :parse_args
)
set COM_PORT=%~1
shift
goto :parse_args
:args_done

cd /d "%~dp0.."

REM Build if needed
if not exist ".pio\build\esp32dev\firmware.bin" (
    echo Building firmware first...
    pio run -e esp32dev
    if errorlevel 1 (
        echo Build failed!
        exit /b 1
    )
)
echo.

REM Try UART OTA first (unless --manual specified)
if %SKIP_OTA%==1 goto :manual_upload

echo ============================================================
echo   Attempting UART OTA upload to %COM_PORT%
echo   (If ESP32 is running OTA-capable firmware)
echo ============================================================
echo.

python tools\uart_ota.py -p %COM_PORT%
if %ERRORLEVEL% EQU 0 (
    echo.
    echo OTA upload successful!
    goto :done
)

echo.
echo UART OTA failed or not supported. Falling back to manual bootloader...
echo.

:manual_upload
echo ============================================================
echo   Manual Bootloader Upload to %COM_PORT%
echo ============================================================
echo.
echo Put ESP32 in download mode during countdown:
echo   Hold BOOT, press RESET, release RESET, release BOOT.
echo.
python tools\waveshare_uploader.py -p %COM_PORT% --build-dir .pio\build\esp32dev --manual

:done
endlocal
pause

@echo off
REM Launch Pan-Tilt Board Test Utility GUI

cd /d %~dp0\..

echo Starting Pan-Tilt Board Test Utility...
python tools\pan_tilt_test_gui.py

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: Failed to start GUI
    echo Make sure Python and pyserial are installed:
    echo   pip install pyserial
    pause
)

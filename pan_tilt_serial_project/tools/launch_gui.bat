@echo off
echo Starting Pan-Tilt Test GUI...
cd /d %~dp0\..

REM Try pythonw first (better for GUI), fallback to python
pythonw tools\pan_tilt_test_gui.py 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo pythonw failed, trying python...
    python tools\pan_tilt_test_gui.py
)

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: Failed to start GUI
    echo Make sure Python and pyserial are installed:
    echo   pip install pyserial
    pause
)

@echo off
title Pan-Tilt Test GUI
echo ========================================
echo Starting Pan-Tilt Test GUI...
echo ========================================
cd /d "%~dp0"
python pan_tilt_test_gui.py
if errorlevel 1 (
    echo.
    echo ERROR: Failed to start GUI
    echo.
    pause
)

@echo off
set BACKEND=%~dp0..\backend
echo Starting backend from %BACKEND%...
start "Gimbal Backend" cmd /k "cd /d "%BACKEND%" && python app.py"
timeout /t 3 /nobreak >nul
start http://localhost:5000
echo Browser opened. Close the backend window to stop the server.

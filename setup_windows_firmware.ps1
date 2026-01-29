# Windows setup for Waveshare Pan-Tilt firmware updates (ESP32)
#
# What it does:
# - Ensures pip is usable
# - Installs Python packages needed for USB control + firmware flashing
#
# Run (PowerShell):
#   Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
#   .\setup_windows_firmware.ps1

$ErrorActionPreference = "Stop"

function Write-Step($msg) {
  Write-Host ""
  Write-Host "==> $msg"
}

Write-Step "Checking Python"
try {
  $py = Get-Command python -ErrorAction Stop
} catch {
  Write-Host "Python was not found in PATH."
  Write-Host "Install Python 3 from the Microsoft Store or from https://www.python.org/downloads/"
  exit 1
}

Write-Step "Python version"
python --version

Write-Step "Upgrading pip (best-effort)"
python -m pip install --upgrade pip | Out-Host

Write-Step "Installing USB control deps (requirements.txt)"
python -m pip install -r ".\requirements.txt" | Out-Host

Write-Step "Installing firmware flashing deps (requirements_firmware.txt)"
python -m pip install -r ".\requirements_firmware.txt" | Out-Host

Write-Step "Listing serial ports (for convenience)"
python -m serial.tools.list_ports | Out-Host

Write-Step "Done"
Write-Host "Next:"
Write-Host "1) Ensure the CP210x driver is installed if no COM port appears."
Write-Host "2) Download the Waveshare firmware binaries (see FIRMWARE_UPDATE.md)."
Write-Host "3) Flash with: python .\flash_firmware.py --port COM9 --image path\\to\\firmware.bin --erase"


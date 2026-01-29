# Script to help download and install CP2102 driver
# This script opens the download page and provides installation instructions

Write-Host "CP2102 Driver Download Helper" -ForegroundColor Cyan
Write-Host "================================" -ForegroundColor Cyan
Write-Host ""

# Check if driver is already installed
Write-Host "Checking for existing CP2102 driver..." -ForegroundColor Yellow
$ports = Get-PnpDevice -Class Ports -Status OK | Where-Object { $_.FriendlyName -like "*CP210*" }
if ($ports) {
    Write-Host "Found CP2102 device(s):" -ForegroundColor Green
    $ports | ForEach-Object { Write-Host "  - $($_.FriendlyName)" -ForegroundColor Green }
} else {
    Write-Host "No CP2102 device found in Device Manager." -ForegroundColor Yellow
    Write-Host "Make sure your ESP32 board is connected via USB." -ForegroundColor Yellow
}
Write-Host ""

# Open download page
Write-Host "Opening Silicon Labs driver download page..." -ForegroundColor Cyan
Write-Host "URL: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers" -ForegroundColor Cyan
Start-Process "https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers"

Write-Host ""
Write-Host "Instructions:" -ForegroundColor Yellow
Write-Host "1. Download the Windows driver package from the page that opened" -ForegroundColor White
Write-Host "2. Extract the ZIP file" -ForegroundColor White
Write-Host "3. Run CP210xVCPInstaller_x64.exe (for 64-bit Windows)" -ForegroundColor White
Write-Host "4. Follow the installation wizard" -ForegroundColor White
Write-Host "5. Unplug and replug your ESP32 board" -ForegroundColor White
Write-Host ""
Write-Host "After installation, run this script again to verify the driver." -ForegroundColor Cyan

# Alternative: Try to download Waveshare's driver if available
Write-Host ""
Write-Host "Alternative: Waveshare driver (if Silicon Labs site requires registration):" -ForegroundColor Yellow
Write-Host "https://files.waveshare.com/upload/d/d6/CP210x_USB_TO_UART.zip" -ForegroundColor Cyan

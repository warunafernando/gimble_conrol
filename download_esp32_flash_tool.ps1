# ESP32 Flash Download Tool - Download Helper

Write-Host "ESP32 Flash Download Tool" -ForegroundColor Cyan
Write-Host "========================" -ForegroundColor Cyan
Write-Host ""

Write-Host "What is ESP32 Flash Download Tool?" -ForegroundColor Yellow
Write-Host "-----------------------------------" -ForegroundColor Yellow
Write-Host "A Windows GUI tool from Espressif that can flash ESP32 firmware." -ForegroundColor White
Write-Host "It's often MORE RELIABLE than esptool because:" -ForegroundColor Green
Write-Host "  1. Works even when DTR/RTS signals don't work properly" -ForegroundColor White
Write-Host "  2. Can detect ESP32 already in download mode (manual entry)" -ForegroundColor White
Write-Host "  3. Better retry logic and connection handling" -ForegroundColor White
Write-Host "  4. GUI interface with visual feedback" -ForegroundColor White
Write-Host ""

Write-Host "Why it works when esptool doesn't:" -ForegroundColor Yellow
Write-Host "  - esptool relies on DTR/RTS to auto-enter download mode" -ForegroundColor White
Write-Host "  - If CP2102 driver doesn't handle DTR/RTS, esptool fails" -ForegroundColor White
Write-Host "  - Flash Download Tool can work with manual download mode entry" -ForegroundColor White
Write-Host ""

Write-Host "Download Options:" -ForegroundColor Yellow
Write-Host "-----------------" -ForegroundColor Yellow
Write-Host "1. Official Espressif website:" -ForegroundColor Cyan
Write-Host "   https://www.espressif.com/en/tools-type/flash-download-tools" -ForegroundColor White
Write-Host ""
Write-Host "2. Direct download (if available):" -ForegroundColor Cyan
Write-Host "   https://dl.espressif.com/public/flash_download_tool.zip" -ForegroundColor White
Write-Host ""

# Try to open the download page
Write-Host "Opening Espressif download page..." -ForegroundColor Cyan
Start-Process "https://www.espressif.com/en/tools-type/flash-download-tools"

Write-Host ""
Write-Host "After downloading:" -ForegroundColor Yellow
Write-Host "1. Extract the ZIP file" -ForegroundColor White
Write-Host "2. Run flash_download_tools.exe" -ForegroundColor White
Write-Host "3. Select ESP32 chip type" -ForegroundColor White
Write-Host "4. Configure firmware files (see ESP32_FLASH_TOOL_GUIDE.md)" -ForegroundColor White
Write-Host "5. Put ESP32 in download mode (hold BOOT, press RESET, release BOOT)" -ForegroundColor White
Write-Host "6. Click START to flash" -ForegroundColor White
Write-Host ""
Write-Host "See ESP32_FLASH_TOOL_GUIDE.md for detailed instructions." -ForegroundColor Cyan

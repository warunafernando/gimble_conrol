# ESP32 Flash Script with Manual Download Mode
# Put ESP32 in download mode FIRST, then run this script

$port = "COM9"
$projectDir = "E:\Arduino_Controller\pan_tilt_base_v0.9"
$buildDir = "$projectDir\.pio\build\esp32dev"
$esptool = "$env:USERPROFILE\.platformio\packages\tool-esptoolpy\esptool.py"
$bootApp0 = "$env:USERPROFILE\.platformio\packages\framework-arduinoespressif32\tools\partitions\boot_app0.bin"

Write-Host "IMPORTANT: Hold BOOT button, press RESET, KEEP HOLDING BOOT" -ForegroundColor Yellow
Write-Host "Starting upload in 3 seconds..." -ForegroundColor Yellow
Start-Sleep -Seconds 3

Write-Host "Connecting and uploading (keep holding BOOT if needed)..." -ForegroundColor Green

$maxRetries = 3
$retryCount = 0
$success = $false

while ($retryCount -lt $maxRetries -and -not $success) {
    $retryCount++
    Write-Host "Attempt $retryCount of $maxRetries..." -ForegroundColor Cyan
    
    python "$esptool" --chip esp32 --port $port --baud 115200 `
        --before no_reset --after hard_reset `
        write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect `
        0x1000 "$buildDir\bootloader.bin" `
        0x8000 "$buildDir\partitions.bin" `
        0xe000 "$bootApp0" `
        0x10000 "$buildDir\firmware.bin"
    
    if ($LASTEXITCODE -eq 0) {
        $success = $true
    } else {
        if ($retryCount -lt $maxRetries) {
            Write-Host "Retrying in 1 second... (make sure ESP32 is still in download mode)" -ForegroundColor Yellow
            Start-Sleep -Seconds 1
        }
    }
}

if ($LASTEXITCODE -eq 0) {
    Write-Host "Upload successful!" -ForegroundColor Green
} else {
    Write-Host "Upload failed. Make sure ESP32 is in download mode." -ForegroundColor Red
}

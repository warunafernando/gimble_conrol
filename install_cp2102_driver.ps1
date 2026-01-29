# CP2102 Driver Installation Script
# This script installs the downloaded Waveshare CP2102 driver

$driverPath = "$env:TEMP\CP210x_Driver\CP210x_USB_TO_UART\CP210x_WIN10??"

Write-Host "CP2102 Driver Installation" -ForegroundColor Cyan
Write-Host "===========================" -ForegroundColor Cyan
Write-Host ""

# Find the Windows 10 installer
$installer64 = Get-ChildItem -Path "$env:TEMP\CP210x_Driver" -Recurse -Filter "CP210xVCPInstaller_x64.exe" | Select-Object -First 1
$installer86 = Get-ChildItem -Path "$env:TEMP\CP210x_Driver" -Recurse -Filter "CP210xVCPInstaller_x86.exe" | Select-Object -First 1

if (-not $installer64 -and -not $installer86) {
    Write-Host "ERROR: Installer not found!" -ForegroundColor Red
    Write-Host "Please download the driver first by running: download_cp2102_driver.ps1" -ForegroundColor Yellow
    exit 1
}

# Determine which installer to use (prefer 64-bit)
$installer = $installer64
$arch = "64-bit"
if (-not $installer) {
    $installer = $installer86
    $arch = "32-bit"
}

Write-Host "Found installer: $($installer.FullName)" -ForegroundColor Green
Write-Host "Architecture: $arch" -ForegroundColor Green
Write-Host ""

# Check if running as administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)

if (-not $isAdmin) {
    Write-Host "WARNING: This script needs administrator privileges to install the driver." -ForegroundColor Yellow
    Write-Host "Attempting to restart with administrator privileges..." -ForegroundColor Yellow
    Write-Host ""
    
    # Restart script as administrator
    $scriptPath = $MyInvocation.MyCommand.Path
    Start-Process powershell -Verb RunAs -ArgumentList "-ExecutionPolicy Bypass -File `"$scriptPath`""
    exit
}

Write-Host "Installing CP2102 driver..." -ForegroundColor Cyan
Write-Host "This may take a moment..." -ForegroundColor Yellow
Write-Host ""

# Run the installer
try {
    $process = Start-Process -FilePath $installer.FullName -ArgumentList "/S" -Wait -PassThru -NoNewWindow
    
    if ($process.ExitCode -eq 0) {
        Write-Host "Driver installation completed successfully!" -ForegroundColor Green
    } else {
        Write-Host "Installation completed with exit code: $($process.ExitCode)" -ForegroundColor Yellow
    }
} catch {
    Write-Host "ERROR: Failed to run installer" -ForegroundColor Red
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Next steps:" -ForegroundColor Yellow
Write-Host "1. Unplug your ESP32 board" -ForegroundColor White
Write-Host "2. Wait 5 seconds" -ForegroundColor White
Write-Host "3. Plug it back in" -ForegroundColor White
Write-Host "4. Check Device Manager to verify the driver is working" -ForegroundColor White
Write-Host ""
Write-Host "After replugging, try flashing the firmware again." -ForegroundColor Cyan

# Installation Guide

## Quick Setup

1. **Install Python dependencies:**
   ```bash
   pip install -r tools/requirements.txt
   ```

   Or install individually:
   ```bash
   pip install esptool pyserial
   ```

## Verification

Test the installation:
```bash
python tools/waveshare_uploader.py --help
```

You should see the help message. If you get "esptool not found", install it as above.

## PlatformIO Integration

The tool is automatically integrated into PlatformIO. Just run:
```bash
pio run -t upload --upload-port COM9
```

The tool will automatically:
- Detect the build directory
- Upload bootloader, partitions, and firmware
- Use proper DTR/RTS timing for the Waveshare board

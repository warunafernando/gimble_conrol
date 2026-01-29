Import("env")
import time

def pre_upload_dtr_rts(source, target, env):
    """
    Pre-upload action: Activate auto-download circuit using DTR/RTS signals.
    This gives the ESP32 time to enter download mode before esptool connects.
    """
    upload_port = env.get("UPLOAD_PORT")
    if not upload_port:
        upload_port = "COM9"  # Default for this project
    
    print("=" * 60)
    print("Activating Waveshare auto-download circuit...")
    print("=" * 60)
    
    try:
        import serial
        ser = serial.Serial(upload_port, 115200, timeout=1)
        
        # Sequence to trigger auto-download circuit (mimics Waveshare tool behavior)
        print("Sending DTR/RTS sequence...")
        ser.setDTR(False)  # DTR low = EN low (reset)
        ser.setRTS(True)   # RTS high = GPIO0 high  
        time.sleep(0.1)
        
        ser.setDTR(True)   # DTR high = EN high (release reset)
        ser.setRTS(False)  # RTS low = GPIO0 low (download mode)
        time.sleep(0.1)
        
        ser.setDTR(False)  # DTR low = EN low (reset again)
        time.sleep(0.1)
        
        ser.setDTR(True)   # DTR high = EN high (release reset, now in download mode)
        ser.close()
        # Wait for OS to release port so upload script can open it (Windows)
        print("Waiting for ESP32 to enter download mode...")
        time.sleep(0.6)  # Give circuit time to stabilize (critical timing!)
        print("Ready for upload")
        time.sleep(0.8)   # Let Windows release COM port before upload script runs
        print()
        
    except ImportError:
        print("Warning: pyserial not available, skipping DTR/RTS sequence")
        print("You may need to manually put ESP32 in download mode")
        time.sleep(0.5)
    except Exception as e:
        print(f"Warning: Could not control serial port: {e}")
        print("You may need to manually put ESP32 in download mode")
        print("(Hold BOOT, press RESET, release BOOT)")
        time.sleep(0.5)

# Disabled: uploader does DTR/RTS so port isn't held by pre_upload when upload runs
# env.AddPreAction("upload", pre_upload_dtr_rts)

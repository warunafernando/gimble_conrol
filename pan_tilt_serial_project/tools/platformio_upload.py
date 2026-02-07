"""
PlatformIO integration script for Waveshare uploader
Add this to platformio.ini: extra_scripts = tools/platformio_upload.py
"""

Import("env")
import subprocess
import sys
from pathlib import Path

def upload_with_waveshare(source, target, env):
    """Use Waveshare uploader instead of default esptool"""
    
    # Get paths
    firmware = str(source[0])
    port = env.get("UPLOAD_PORT", "COM9")
    build_dir = env.get("BUILD_DIR")
    
    # Find uploader script (PROJECT_DIR when __file__ not set in SCons)
    try:
        script_dir = Path(__file__).resolve().parent
    except NameError:
        script_dir = Path(env.get("PROJECT_DIR", ".")) / "tools"
    uploader_script = script_dir / "waveshare_uploader.py"
    
    if not uploader_script.exists():
        print(f"ERROR: Uploader script not found: {uploader_script}")
        print("Falling back to default upload method")
        return
    
    # Build command
    cmd = [
        sys.executable,
        str(uploader_script),
        "-p", port,
        "--build-dir", build_dir
    ]
    if env.GetProjectOption("upload_swap_dtr_rts", "0") == "1":
        cmd.append("--swap-dtr-rts")
    
    print("=" * 60)
    print("Using Waveshare-Compatible Uploader")
    print("=" * 60)
    print(f"Command: {' '.join(cmd)}")
    print()
    
    # Execute
    try:
        result = env.Execute(" ".join(cmd))
        if result != 0:
            print("\nUpload failed!")
            return
    except Exception as e:
        print(f"\nERROR: {e}")
        print("Falling back to default upload method")

# Replace default upload command
env.Replace(UPLOADCMD=upload_with_waveshare)

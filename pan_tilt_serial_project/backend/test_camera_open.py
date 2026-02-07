"""Quick test: open camera, get one frame, report result."""
import sys
import time

# run from backend dir so imports work
sys.path.insert(0, ".")
import camera

def main():
    print("1. Listing cameras...")
    cams = camera.list_cameras()
    if not cams:
        print("   No cameras found.")
        return 1
    for c in cams:
        print(f"   Index {c['index']} backend={c['backend']} {c.get('width',0)}x{c.get('height',0)} @ {c.get('fps',0)} fps")
    idx = cams[0]["index"]
    print(f"\n2. Starting camera at index {idx}...")
    ok = camera.start_camera(cam_index=idx)
    if not ok:
        print("   start_camera() returned False.")
        return 1
    print("   Camera thread started, waiting for first frame...")
    for i in range(40):
        frame = camera.get_frame()
        if frame:
            print(f"   Got frame ({len(frame)} bytes) after ~{i*0.25:.1f}s")
            break
        time.sleep(0.25)
    else:
        print("   No frame received within 10s.")
        camera.stop_camera()
        return 1
    print("\n3. Stopping camera...")
    camera.stop_camera()
    time.sleep(1.0)
    print("   Done. Camera open test PASSED.")
    return 0

if __name__ == "__main__":
    sys.exit(main())

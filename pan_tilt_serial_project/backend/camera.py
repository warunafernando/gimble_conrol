"""
Single dedicated thread for Arducam capture with AprilTag detection.
Serves frames for MJPEG stream.
"""

import threading
import platform
from typing import Optional, List, Dict

_cap_thread: Optional["CaptureThread"] = None
_cap_lock = threading.Lock()

try:
    import cv2
    import numpy as np
    CV2_AVAILABLE = True
except ImportError:
    CV2_AVAILABLE = False
    cv2 = None
    np = None

try:
    from pupil_apriltags import Detector
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False
    Detector = None

CAP_BACKEND = cv2.CAP_DSHOW if platform.system() == "Windows" and cv2 else 0
# Windows: also try MSMF (Microsoft Media Foundation) - some USB cams need it
CAP_MSMF = getattr(cv2, "CAP_MSMF", 1400) if cv2 else 0

# Backends to try - try both orders: DSHOW first (Arducam) and CAP_ANY first (some USB cams)
def _backends():
    if not cv2:
        return []
    if platform.system() == "Windows":
        return [cv2.CAP_ANY, cv2.CAP_DSHOW, CAP_MSMF]
    return [cv2.CAP_ANY]


def _try_open(index, backend, retries: int = 12, delay: float = 0.8):
    """Open camera at index (int or str) with backend; retry read() with delay (USB needs time)."""
    import time
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        return None
    # Some USB cameras need time after open before first read
    time.sleep(1.2)
    for _ in range(retries):
        ret, frame = cap.read()
        if ret and frame is not None:
            return cap
        time.sleep(delay)
    cap.release()
    return None


def _open_camera(prefer_index: int = 0):
    """Try to open a camera: prefer_index first, then 0-9. Returns (cap, actual_index) or (None, -1)."""
    if not CV2_AVAILABLE:
        return None, -1
    backends = _backends()
    # Try preferred index first
    for backend in backends:
        cap = _try_open(prefer_index, backend)
        if cap is not None:
            return cap, prefer_index
    # Try indices 0-9
    for idx in range(10):
        if idx == prefer_index:
            continue
        for backend in backends:
            cap = _try_open(idx, backend)
            if cap is not None:
                return cap, idx
    return None, -1


def _get_camera_props(cap) -> Dict:
    """Read resolution, fps, and output format from an open VideoCapture."""
    if not cv2:
        return {}
    try:
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        fps_val = cap.get(cv2.CAP_PROP_FPS)
        fps = round(float(fps_val), 1) if fps_val == fps_val and fps_val is not None else 0
        fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC) or 0)
        if fourcc_int < 0:
            fourcc_int = 0
        format_str = "".join(chr((fourcc_int >> (8 * i)) & 0xFF) for i in range(4)).strip()
        if not format_str or not all(32 <= ord(c) <= 126 for c in format_str):
            format_str = "0x%08X" % (fourcc_int & 0xFFFFFFFF)
    except Exception:
        format_str = "?"
        w = h = 0
        fps = 0
    return {"width": w, "height": h, "fps": fps, "fourcc": format_str}


def list_cameras():
    """Return list of cameras with index, backend, resolution, fps, and output format."""
    if not CV2_AVAILABLE:
        return []
    backend_names = {
        cv2.CAP_ANY: "ANY",
        cv2.CAP_DSHOW: "DSHOW",
        CAP_MSMF: "MSMF",
    }
    found = []
    seen = set()
    for idx in range(10):
        for backend in _backends():
            for key in (idx, str(idx)):
                cap = _try_open(key, backend, retries=8, delay=0.5)
                if cap is not None:
                    name = backend_names.get(backend, str(backend))
                    if (idx, name) not in seen:
                        seen.add((idx, name))
                        props = _get_camera_props(cap)
                        found.append({
                            "index": idx,
                            "backend": name,
                            "width": props.get("width", 0),
                            "height": props.get("height", 0),
                            "fps": props.get("fps", 0),
                            "fourcc": props.get("fourcc", "?"),
                        })
                    cap.release()
                    break
            if any(f["index"] == idx for f in found):
                break
    return found


# Modes to probe when reading supported res/fps from camera (includes 1200p @ 50fps)
_CANDIDATE_MODES = [
    (320, 240), (640, 480), (800, 600), (960, 600),
    (1280, 720), (1600, 1200), (1920, 1080), (1920, 1200),  # 1200p
]
_CANDIDATE_FPS = [15, 24, 25, 30, 50]


def get_camera_modes(cam_index: int) -> List[Dict]:
    """Probe camera at index for supported (width, height, fps) modes. Returns list of {width, height, fps}."""
    if not CV2_AVAILABLE:
        return []
    import time
    cap = None
    for backend in _backends():
        cap = _try_open(cam_index, backend, retries=6, delay=0.5)
        if cap is not None:
            break
    if cap is None:
        return []
    modes = []
    for (w, h) in _CANDIDATE_MODES:
        for fps in _CANDIDATE_FPS:
            try:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
                cap.set(cv2.CAP_PROP_FPS, fps)
                time.sleep(0.1)
                for _ in range(3):
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        modes.append({"width": w, "height": h, "fps": fps})
                        break
                    time.sleep(0.08)
            except Exception:
                pass
    cap.release()
    return modes


class CaptureThread(threading.Thread):
    """Single dedicated thread: open camera, detect AprilTags, store latest JPEG."""

    def __init__(self, cam_index: int = 0, width: int = 960, height: int = 600, fps: float = 30.0, quality: int = 85):
        super().__init__(daemon=True)
        self.cam_index = cam_index
        self.width = width
        self.height = height
        self.fps = fps
        self.quality = quality
        self._lock = threading.Lock()
        self._latest_jpeg: Optional[bytes] = None
        self._latest_tags: List[Dict] = []
        self._running = True
        self._ok = False
        self._detector = None
        self._actual_index = -1  # index we actually opened

    def run(self) -> None:
        if not CV2_AVAILABLE:
            return
        cap, self._actual_index = _open_camera(self.cam_index)
        if cap is None:
            import sys
            print("[Camera] No camera found (tried indices 0-4 with DSHOW/default)", file=sys.stderr)
            return
        if self._actual_index != self.cam_index:
            import sys
            print(f"[Camera] Preferred index {self.cam_index} failed, using index {self._actual_index}", file=sys.stderr)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        if self.fps > 0:
            cap.set(cv2.CAP_PROP_FPS, self.fps)
        encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]

        # Initialize AprilTag detector
        if APRILTAG_AVAILABLE:
            self._detector = Detector(
                families="tag36h11",  # Common tag family
                nthreads=2,
                quad_decimate=2.0,  # Faster detection
                quad_sigma=0.0,
                refine_edges=1,
                decode_sharpening=0.25,
            )

        while self._running and cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                continue

            tags_info = []
            
            # Detect AprilTags
            if self._detector is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                detections = self._detector.detect(gray)
                
                for det in detections:
                    # Extract tag info
                    tag_id = det.tag_id
                    center = det.center
                    corners = det.corners
                    
                    # Draw tag on frame
                    corners_int = corners.astype(int)
                    # Draw outline
                    cv2.polylines(frame, [corners_int], True, (0, 255, 0), 2)
                    # Draw center
                    cx, cy = int(center[0]), int(center[1])
                    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                    # Draw ID
                    cv2.putText(frame, f"ID:{tag_id}", (cx - 20, cy - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                    
                    # Store tag info
                    tags_info.append({
                        "id": tag_id,
                        "center_x": round(center[0], 1),
                        "center_y": round(center[1], 1),
                        "corners": corners.tolist(),
                    })

            # Encode frame
            _, buf = cv2.imencode(".jpg", frame, encode_params)
            if buf is not None:
                with self._lock:
                    self._latest_jpeg = buf.tobytes()
                    self._latest_tags = tags_info
                if not self._ok:
                    self._ok = True  # Mark ready only after first frame

        cap.release()
        self._ok = False

    def stop(self) -> None:
        self._running = False

    def get_frame(self) -> Optional[bytes]:
        with self._lock:
            return self._latest_jpeg

    def get_tags(self) -> List[Dict]:
        with self._lock:
            return self._latest_tags.copy()

    def is_ok(self) -> bool:
        return self._ok


def start_camera(cam_index: int = 0, width: int = 960, height: int = 600, fps: float = 30.0) -> bool:
    """Start camera capture thread. Returns True if started."""
    global _cap_thread
    if not CV2_AVAILABLE:
        return False
    with _cap_lock:
        if _cap_thread and _cap_thread.is_alive():
            return _cap_thread.is_ok()
        _cap_thread = CaptureThread(cam_index=cam_index, width=width, height=height, fps=fps)
        _cap_thread.start()
        # Wait for first frame (open + first read can take 10+ s on some USB cams)
        import time
        for _ in range(100):
            time.sleep(0.25)
            if _cap_thread.get_frame():
                return True
            if not _cap_thread.is_alive():
                break
        return _cap_thread.is_ok()


def stop_camera() -> None:
    """Stop camera capture thread."""
    global _cap_thread
    with _cap_lock:
        if _cap_thread:
            _cap_thread.stop()
            _cap_thread.join(timeout=2)
            _cap_thread = None


def get_frame() -> Optional[bytes]:
    """Get latest JPEG frame. Returns None if not available."""
    with _cap_lock:
        if _cap_thread:
            return _cap_thread.get_frame()
    return None


def get_tags() -> List[Dict]:
    """Get latest detected AprilTags."""
    with _cap_lock:
        if _cap_thread:
            return _cap_thread.get_tags()
    return []

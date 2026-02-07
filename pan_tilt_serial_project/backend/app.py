"""
PC backend: serial bridge to ESP32 (binary protocol), WebSocket push to web GUI.
Debug: last N log lines and connection status in debug panel.
"""

import os
import threading
import time
from queue import Queue, Full

from flask import Flask, Response, request, send_from_directory
from flask_socketio import SocketIO, emit

from typing import Optional, Tuple

# All socketio.emit from serial thread are queued; single thread drains and emits (avoids crash)
_emit_queue = Queue(maxsize=400)

try:
    from camera import start_camera, stop_camera, get_frame, get_tags, list_cameras, get_camera_modes
    CAMERA_AVAILABLE = True
except (ImportError, Exception):
    CAMERA_AVAILABLE = False
    start_camera = lambda *a, **k: False
    stop_camera = lambda: None
    get_frame = lambda: None
    get_tags = lambda: []
    list_cameras = lambda: []
    get_camera_modes = lambda idx: []
import serial.tools.list_ports
from serial_bridge import DebugLog, SerialBridge
from protocol import (
    CMD_GET_FW_INFO,
    CMD_I2C_SCAN,
    RSP_ACK_EXECUTED,
    RSP_FW_INFO,
    RSP_I2C_SCAN,
    decode_fw_info,
    decode_i2c_scan,
    decode_imu,
    decode_imu2,
    decode_ina,
    decode_move_feedback,
)

app = Flask(__name__, static_folder="static", static_url_path="")
app.config["SECRET_KEY"] = "gimbal-backend"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

debug_log = DebugLog(max_lines=500)
bridge: Optional[SerialBridge] = None
_seq = 0
_state = {"connected": False, "port": "", "last_move": None, "last_imu": None, "last_imu2": None, "last_ina": None, "last_gimbal_state": None, "fw_info": None}
# Rate-limit IMU frame emits so fast reads don't overwhelm server (min interval in seconds)
_imu_emit_interval = 0.05  # 50 ms
_last_imu_emit_time = 0.0
_last_imu2_emit_time = 0.0
_frame_emit_lock = threading.Lock()
# Rate-limit IMU commands (126, 127) so rapid button clicks don't flood serial and crash server
CMD_GET_IMU = 126
CMD_GET_IMU2 = 127
_imu_cmd_interval = 0.12  # 120 ms min between Get IMU / Get IMU2
_last_imu_cmd_time = 0.0
_imu_cmd_lock = threading.Lock()


def next_seq() -> int:
    global _seq
    _seq = (_seq + 1) & 0xFFFF
    return _seq


def on_frame(seq: int, type_id: int, payload: bytes) -> None:
    """Runs in serial thread: decode, update state, queue for emit. No socketio here."""
    try:
        data = {"seq": seq, "type": type_id}
        if type_id == RSP_ACK_EXECUTED and len(payload) >= 8:
            fb = decode_move_feedback(payload)
            if fb:
                _state["last_move"] = fb
                data["payload"] = fb
        elif type_id == 1002 and len(payload) >= 50:
            try:
                imu = decode_imu(payload)
                if imu:
                    _state["last_imu"] = imu
                    data["payload"] = imu
            except Exception:
                pass
        elif type_id == 1003 and len(payload) >= 28:
            try:
                imu2 = decode_imu2(payload)
            except Exception:
                imu2 = None
            if imu2:
                _state["last_imu2"] = imu2
                data["payload"] = imu2
        elif type_id == 1010:
            if len(payload) >= 21:
                try:
                    ina = decode_ina(payload)
                    if ina:
                        _state["last_ina"] = ina
                        data["payload"] = ina
                    else:
                        data["payload"] = {"error": "decode_failed", "len": len(payload)}
                except Exception:
                    data["payload"] = {"error": "decode_exception", "len": len(payload)}
            else:
                data["payload"] = {"error": "payload_too_short", "len": len(payload), "expected": 21}
        elif type_id == 1013 and len(payload) >= 1:
            s = payload[0]
            names = {0: "idle", 1: "tracking", 2: "config"}
            gs = {"state": s, "stateName": names.get(s, "?")}
            _state["last_gimbal_state"] = gs
            data["payload"] = gs
        elif type_id == RSP_FW_INFO:
            fw = decode_fw_info(payload)
            if fw:
                _state["fw_info"] = fw
                data["payload"] = fw
        elif type_id == RSP_I2C_SCAN:
            i2c = decode_i2c_scan(payload)
            if i2c:
                data["payload"] = i2c
        try:
            _emit_queue.put_nowait(("frame", data))
        except Full:
            pass
    except Exception as e:
        debug_log.append(f"on_frame error type={type_id} err={e}")


def _on_log(msg: str) -> None:
    try:
        ts = time.strftime("%H:%M:%S", time.localtime())
        line = f"[{ts}] {msg}"
        try:
            _emit_queue.put_nowait(("log", line))
        except Full:
            pass
    except Exception:
        pass


def _emit_worker() -> None:
    """Single thread that drains _emit_queue and does all socketio.emit (avoids serial-thread emit crash)."""
    global _last_imu_emit_time, _last_imu2_emit_time
    while True:
        try:
            kind, payload = _emit_queue.get()
            if kind is None:
                break
            with app.app_context():
                if kind == "log":
                    socketio.emit("log_line", {"line": payload})
                elif kind == "frame":
                    data = payload
                    type_id = data.get("type")
                    # Rate-limit IMU/IMU2 frame emits
                    now = time.monotonic()
                    do_emit = True
                    if type_id == 1002:
                        with _frame_emit_lock:
                            if now - _last_imu_emit_time < _imu_emit_interval:
                                do_emit = False
                            else:
                                _last_imu_emit_time = now
                    elif type_id == 1003:
                        with _frame_emit_lock:
                            if now - _last_imu2_emit_time < _imu_emit_interval:
                                do_emit = False
                            else:
                                _last_imu2_emit_time = now
                    if do_emit:
                        socketio.emit("frame", data)
                    if type_id == 1003 and data.get("payload"):
                        socketio.emit("imu2", data["payload"])
                    elif type_id == RSP_FW_INFO and data.get("payload"):
                        socketio.emit("fw_info", data["payload"])
                    elif type_id == RSP_I2C_SCAN and data.get("payload"):
                        socketio.emit("i2c_scan", data["payload"])
        except Exception as e:
            debug_log.append(f"emit_worker error: {e}")


def run_bridge(port: str, baud: int = 921600) -> Tuple[bool, Optional[str]]:
    global bridge
    if bridge:
        bridge.stop()
        bridge = None
    bridge = SerialBridge(
        port=port,
        baud=baud,
        on_frame=on_frame,
        on_log=_on_log,
        debug_log=debug_log,
    )
    ok, err_msg = bridge.start()
    if ok:
        _state["connected"] = True
        _state["port"] = port
        _state["fw_info"] = None
        socketio.emit("state", _state)
        bridge.send_command(next_seq(), CMD_GET_FW_INFO, b"")
        return True, None
    _state["connected"] = False
    _state["port"] = ""
    socketio.emit("state", _state)
    return False, err_msg


@app.route("/api/serial_ports")
def api_serial_ports():
    """List available serial port names (e.g. COM9) for GUI connection."""
    try:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return {"ok": True, "ports": sorted(ports)}
    except Exception as e:
        return {"ok": False, "error": str(e), "ports": []}


@app.route("/")
def index():
    r = send_from_directory("static", "index.html")
    r.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    return r


# Camera index and stream settings (default 1200p @ 50 fps)
CAMERA_INDEX = 0
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1200
CAMERA_FPS = 50.0


def gen_frames():
    """MJPEG generator: yield JPEG frames for /video_feed."""
    import time
    global CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS
    print("[Camera] gen_frames started")
    if CAMERA_AVAILABLE:
        print(f"[Camera] Starting camera index {CAMERA_INDEX} {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS} fps...")
        ok = start_camera(
            cam_index=CAMERA_INDEX,
            width=CAMERA_WIDTH,
            height=CAMERA_HEIGHT,
            fps=CAMERA_FPS,
        )
        if not ok and (CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS) == (1920, 1200, 50.0):
            print("[Camera] 1200p@50 failed, retrying 960x600@30...")
            stop_camera()
            time.sleep(1.5)
            ok = start_camera(cam_index=CAMERA_INDEX, width=960, height=600, fps=30.0)
            if ok:
                CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS = 960, 600, 30.0
                print("[Camera] Fallback 960x600@30 started")
        print(f"[Camera] Camera started: {ok}")
    else:
        print("[Camera] Camera not available (cv2 missing)")
    frame_count = 0
    while True:
        frame = get_frame()
        if frame:
            frame_count += 1
            if frame_count % 100 == 1:
                print(f"[Camera] Yielding frame {frame_count}, size={len(frame)}")
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
        else:
            time.sleep(0.1)


@app.route("/video_feed")
def video_feed():
    """MJPEG stream for live camera feed."""
    return Response(gen_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/cameras")
def api_cameras():
    """List camera indices that open and read a frame."""
    try:
        cams = list_cameras()
        return {"ok": True, "cameras": cams, "count": len(cams)}
    except Exception as e:
        return {"ok": False, "error": str(e), "cameras": [], "count": 0}


@app.route("/api/cameras/current")
def api_cameras_current():
    """Return currently selected camera index and stream settings."""
    return {
        "ok": True,
        "index": CAMERA_INDEX,
        "width": CAMERA_WIDTH,
        "height": CAMERA_HEIGHT,
        "fps": CAMERA_FPS,
    }


@app.route("/api/cameras/<int:index>/modes")
def api_cameras_modes(index):
    """Probe camera at index for supported resolutions and FPS; returns list of {width, height, fps}."""
    if index < 0 or index > 9:
        return {"ok": False, "error": "index must be 0-9", "modes": []}, 400
    if not CAMERA_AVAILABLE:
        return {"ok": False, "error": "camera not available", "modes": []}, 503
    try:
        modes = get_camera_modes(index)
        return {"ok": True, "modes": modes}
    except Exception as e:
        return {"ok": False, "error": str(e), "modes": []}, 500


@app.route("/api/cameras/select", methods=["POST"])
def api_cameras_select():
    """Select camera by index and optional width, height, fps. Stops current stream so next /video_feed uses new settings."""
    global CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_FPS
    try:
        data = request.get_json() or {}
        idx = int(data.get("index", 0))
        if idx < 0 or idx > 9:
            return {"ok": False, "error": "index must be 0-9"}, 400
        CAMERA_INDEX = idx
        if "width" in data and data["width"] is not None:
            w = int(data["width"])
            if 160 <= w <= 4096:
                CAMERA_WIDTH = w
        if "height" in data and data["height"] is not None:
            h = int(data["height"])
            if 120 <= h <= 2160:
                CAMERA_HEIGHT = h
        if "fps" in data and data["fps"] is not None:
            f = float(data["fps"])
            if 1 <= f <= 120:
                CAMERA_FPS = f
        if CAMERA_AVAILABLE:
            stop_camera()
            time.sleep(1.5)  # Let OS release the device before next open
        return {
            "ok": True,
            "index": CAMERA_INDEX,
            "width": CAMERA_WIDTH,
            "height": CAMERA_HEIGHT,
            "fps": CAMERA_FPS,
        }
    except Exception as e:
        return {"ok": False, "error": str(e)}, 400


@app.route("/api/cameras/scan")
def api_cameras_scan():
    """Verbose camera scan: try each index/backend and return what works (HTML for browser)."""
    import sys
    from io import StringIO
    if not CAMERA_AVAILABLE:
        return "<pre>OpenCV/camera not available</pre>", 200
    out = StringIO()
    out.write("Camera scan (indices 0-4, backends ANY/DSHOW/MSMF)...\n\n")
    try:
        cams = list_cameras()
        out.write(f"Found: {len(cams)} camera(s)\n")
        for c in cams:
            out.write(f"  index {c['index']} (backend {c['backend']})\n")
        if not cams:
            out.write("\nNo cameras detected. Try:\n")
            out.write("  1. Close other apps using the camera (browser tabs, Zoom, etc.)\n")
            out.write("  2. Unplug Arducam, wait 5 sec, plug back in\n")
            out.write("  3. Check Device Manager - does the camera appear?\n")
            out.write("  4. Install Arducam driver/software if required for your model\n")
    except Exception as e:
        out.write(f"Error: {e}\n")
    return f"<pre>{out.getvalue()}</pre>", 200


@socketio.on("connect")
def on_connect():
    try:
        emit("state", _state)
        emit("log_history", {"lines": debug_log.get_recent(100)})
        if bridge and bridge._serial and bridge._serial.is_open and _state.get("connected"):
            bridge.send_command(next_seq(), CMD_GET_FW_INFO, b"")
    except Exception as e:
        debug_log.append(f"on_connect error: {e}")


@socketio.on("connect_serial")
def on_connect_serial(data):
    try:
        port = (data.get("port") or "COM3").strip() or "COM3"
        baud = int(data.get("baud", 921600))
        ok, err_msg = run_bridge(port, baud)
        emit("connect_serial_result", {"ok": ok, "port": port, "error": err_msg})
    except Exception as e:
        debug_log.append(f"connect_serial error: {e}")
        emit("connect_serial_result", {"ok": False, "port": data.get("port", ""), "error": str(e)})


@socketio.on("disconnect_serial")
def on_disconnect_serial():
    try:
        global bridge
        if bridge:
            bridge.stop()
            bridge = None
        _state["connected"] = False
        _state["port"] = ""
        socketio.emit("state", _state)
    except Exception as e:
        debug_log.append(f"disconnect_serial error: {e}")


@socketio.on("command")
def on_command(data):
    try:
        _on_command_impl(data)
    except Exception as e:
        debug_log.append(f"command error: {e}")


def _on_command_impl(data):
    global _last_imu_cmd_time
    payload = data.get("payload")
    if payload is None:
        payload = b""
    elif isinstance(payload, list):
        payload = bytes(payload)
    elif isinstance(payload, str):
        import base64
        payload = base64.b64decode(payload)
    seq = data.get("seq")
    if seq is None:
        seq = next_seq()
    else:
        seq = int(seq) & 0xFFFF
    type_id = data.get("type")
    type_id = int(type_id) if type_id is not None else 0
    if not bridge:
        emit("error", {"msg": "Serial not connected or send failed"})
        return
    # Rate-limit Get IMU (126) and Get IMU2 (127) to prevent flood and server crash
    if type_id in (CMD_GET_IMU, CMD_GET_IMU2):
        now = time.monotonic()
        with _imu_cmd_lock:
            if now - _last_imu_cmd_time < _imu_cmd_interval:
                return  # drop duplicate, client still has last IMU from state
            _last_imu_cmd_time = now
    if not bridge.send_command(seq, type_id, payload):
        emit("error", {"msg": "Serial not connected or send failed"})


@socketio.on("get_log")
def on_get_log():
    try:
        emit("log_history", {"lines": debug_log.get_recent(100)})
    except Exception as e:
        debug_log.append(f"get_log error: {e}")


def tag_emitter():
    """Background thread to emit AprilTag detections."""
    import time
    last_tags = []
    while True:
        try:
            time.sleep(0.1)  # 10Hz update rate
            tags = get_tags()
            if tags != last_tags:
                last_tags = tags
                with app.app_context():
                    socketio.emit("apriltags", {"tags": tags})
        except Exception as e:
            debug_log.append(f"tag_emitter error: {e}")


if __name__ == "__main__":
    import sys
    # Camera index: --camera N or CAMERA_INDEX env (default 0)
    if "--camera" in sys.argv:
        i = sys.argv.index("--camera")
        if i + 1 < len(sys.argv):
            CAMERA_INDEX = int(sys.argv[i + 1])
    else:
        CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "0"))
    port_arg = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else None
    if port_arg:
        run_bridge(port_arg)
    # Single thread for all socketio.emit (serial thread only queues; avoids crash on rapid IMU)
    emit_thread = threading.Thread(target=_emit_worker, daemon=True)
    emit_thread.start()
    if CAMERA_AVAILABLE:
        tag_thread = threading.Thread(target=tag_emitter, daemon=True)
        tag_thread.start()
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)

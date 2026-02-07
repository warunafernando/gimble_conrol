"""
PC backend: serial bridge to ESP32 (binary protocol), WebSocket push to web GUI.
On startup: scan COM ports, open each and probe for gimbal model 99; keep first match.
Every 30s: if connected, send GET_FW_INFO and expect response; disconnect if no reply.
"""

import os
import platform
import re
import threading
import time
from queue import Queue, Full

from flask import Flask, Response, request, send_from_directory
from flask_socketio import SocketIO, emit

from typing import Optional, Tuple

import serial


def _com_port_sort_key(port: str) -> Tuple[int, str]:
    """Sort COM ports numerically: COM3, COM4, COM5, COM9, COM13 (not COM13 before COM3)."""
    m = re.match(r"^COM(\d+)$", (port or "").upper())
    return (int(m.group(1)), port) if m else (99999, port)

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
    build_frame,
    parse_frame,
    CMD_GET_FW_INFO,
    CMD_I2C_SCAN,
    RSP_ACK_EXECUTED,
    RSP_ACK_RECEIVED,
    RSP_FW_INFO,
    RSP_I2C_SCAN,
    STX,
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

GIMBAL_MODEL_TARGET = 99
PROBE_TIMEOUT = 2.0
PROBE_SETTLE_AFTER_CLOSE = 1.5
KEEPALIVE_INTERVAL = 30.0
KEEPALIVE_PONG_TIMEOUT = 5.0

debug_log = DebugLog(max_lines=500)
bridge: Optional[SerialBridge] = None
_seq = 0
_state = {
    "connected": False,
    "port": "",
    "gimbal_serial": None,
    "last_move": None,
    "last_imu": None,
    "last_imu2": None,
    "last_ina": None,
    "last_gimbal_state": None,
    "fw_info": None,
    "ping_sent_at": None,
}
_scan_requested = threading.Event()
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


def _serial_port_name(port: str) -> str:
    """On Windows use \\\\.\\COMn for reliable open."""
    port = (port or "").strip()
    if not port:
        return port
    if platform.system() == "Windows":
        pu = port.upper()
        if pu.startswith("COM") and not pu.startswith("\\\\"):
            return "\\\\.\\" + pu
    return port


def probe_port_for_gimbal(port: str, baud: int = 921600) -> Tuple[Optional[dict], Optional[serial.Serial]]:
    """Open port, send GET_FW_INFO, read until RSP_FW_INFO or timeout.
    If model_id==99: return (fw_dict, open_serial) and do NOT close the port.
    Else: return (None, None) and close the port."""
    port_to_use = _serial_port_name(port)
    if not port_to_use:
        return None, None
    ser = None
    try:
        ser = serial.Serial(port_to_use, baud, timeout=0.05)
        ser.reset_input_buffer()
        frame = build_frame(1, CMD_GET_FW_INFO, b"")
        ser.write(frame)
        deadline = time.monotonic() + PROBE_TIMEOUT
        buf = bytearray()
        while time.monotonic() < deadline:
            buf.extend(ser.read(256))
            if len(buf) < 8:
                time.sleep(0.02)
                continue
            try:
                idx = buf.index(STX)
            except ValueError:
                buf.clear()
                continue
            if idx > 0:
                del buf[:idx]
            length = buf[1]
            if length < 4 or length > 251:
                buf.pop(0)
                continue
            frame_size = 4 + length
            if len(buf) < frame_size:
                time.sleep(0.02)
                continue
            frame_bytes = bytes(buf[:frame_size])
            del buf[:frame_size]
            result = parse_frame(frame_bytes)
            if result is None:
                continue
            _seq, type_id, payload = result
            if type_id == RSP_ACK_RECEIVED:
                continue
            if type_id == RSP_FW_INFO:
                fw = decode_fw_info(payload)
                if fw:
                    if fw.get("model_id") == GIMBAL_MODEL_TARGET:
                        open_ser = ser
                        ser = None
                        return (fw, open_ser)
                    debug_log.append(f"Probe {port}: model_id={fw.get('model_id')} (need {GIMBAL_MODEL_TARGET}), skip")
                return None, None
    except (OSError, serial.SerialException) as e:
        debug_log.append(f"Probe {port}: {e}")
        return None, None
    finally:
        if ser and ser.is_open:
            try:
                ser.close()
            except Exception:
                pass
        if ser:
            time.sleep(0.1)
    debug_log.append(f"Probe {port}: no FW_INFO in {PROBE_TIMEOUT}s")
    return None, None


def _probe_one(port: str, result_queue: "Queue", done_count: list) -> None:
    """Probe one port and put (port, fw, ser) on result_queue. done_count[0] += 1 when finished."""
    try:
        fw, open_ser = probe_port_for_gimbal(port)
        result_queue.put((port, fw, open_ser))
    except Exception as e:
        debug_log.append(f"Probe {port} thread: {e}")
        result_queue.put((port, None, None))
    finally:
        done_count[0] += 1


def _auto_connect_worker() -> None:
    """Background: list COM ports, probe all in parallel; use first gimbal (model 99) and keep its port open."""
    time.sleep(1.0)
    debug_log.append("Auto-connect: started")
    while True:
        try:
            if bridge and _state.get("connected"):
                _scan_requested.clear()
                _scan_requested.wait(timeout=30.0)
                continue
            ports = sorted([p.device for p in serial.tools.list_ports.comports()], key=_com_port_sort_key)
            if not ports:
                debug_log.append("Auto-connect: no COM ports found")
                _scan_requested.clear()
                _scan_requested.wait(timeout=5.0)
                continue
            debug_log.append(f"Auto-connect: scanning {ports} (parallel)")
            result_queue = Queue()
            done_count = [0]
            for port in ports:
                t = threading.Thread(target=_probe_one, args=(port, result_queue, done_count), daemon=True)
                t.start()
            chosen_port = None
            chosen_fw = None
            chosen_ser = None
            n = len(ports)

            def drain_rest(remaining: int) -> None:
                for _ in range(remaining):
                    try:
                        port, fw, open_ser = result_queue.get(timeout=3.0)
                        if open_ser and open_ser.is_open:
                            try:
                                open_ser.close()
                            except Exception:
                                pass
                    except Exception:
                        pass

            consumed = 0
            while consumed < n:
                try:
                    port, fw, open_ser = result_queue.get(timeout=1.0)
                    consumed += 1
                    if chosen_port is None and fw and open_ser:
                        chosen_port, chosen_fw, chosen_ser = port, fw, open_ser
                        debug_log.append(f"Auto-connect: {port} is gimbal model 99 serial={fw.get('serial')}, keeping port open")
                        threading.Thread(target=drain_rest, args=(n - consumed,), daemon=True).start()
                        break
                    elif open_ser and open_ser.is_open:
                        try:
                            open_ser.close()
                        except Exception:
                            pass
                except Exception:
                    pass
            if chosen_port is None:
                drain_rest(n - consumed)
            if chosen_port and chosen_fw and chosen_ser:
                ok, err = run_bridge(chosen_port, existing_serial=chosen_ser)
                if ok:
                    _state["gimbal_serial"] = chosen_fw.get("serial")
                    try:
                        _emit_queue.put_nowait(("state", dict(_state)))
                    except Full:
                        pass
                    debug_log.append(f"Auto-connect: connected to {chosen_port} gimbal id {_state['gimbal_serial']}")
                else:
                    debug_log.append(f"Auto-connect: run_bridge({chosen_port}) failed: {err}")
                    try:
                        chosen_ser.close()
                    except Exception:
                        pass
            _scan_requested.clear()
            _scan_requested.wait(timeout=5.0)
        except Exception as e:
            debug_log.append(f"Auto-connect error: {e}")
            _scan_requested.wait(timeout=5.0)


def _keepalive_worker() -> None:
    """Every KEEPALIVE_INTERVAL seconds, if connected, send GET_FW_INFO; if no pong in time, disconnect."""
    global bridge
    while True:
        time.sleep(KEEPALIVE_INTERVAL)
        if not _state.get("connected") or not bridge:
            continue
        try:
            _state["ping_sent_at"] = time.monotonic()
            bridge.send_command(next_seq(), CMD_GET_FW_INFO, b"")
        except Exception:
            pass
        time.sleep(KEEPALIVE_PONG_TIMEOUT)
        if not _state.get("connected"):
            continue
        if _state.get("ping_sent_at") is not None:
            debug_log.append("Keepalive: no response, disconnecting")
            if bridge:
                bridge.stop()
                bridge = None
            _state["connected"] = False
            _state["port"] = ""
            _state["gimbal_serial"] = None
            _state["fw_info"] = None
            _state["ping_sent_at"] = None
            try:
                _emit_queue.put_nowait(("state", dict(_state)))
            except Full:
                pass


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
            _state["ping_sent_at"] = None
            debug_log.append(f"FW_INFO payload len={len(payload)} (69=serial, 70=serial+model_id)")
            fw = decode_fw_info(payload)
            if fw:
                fw["_payload_len"] = len(payload)
                debug_log.append(f"FW_INFO decoded serial={fw.get('serial')!r} model_id={fw.get('model_id')!r} version_a={fw.get('version_a')!r}")
                _state["fw_info"] = dict(fw)
                _state["gimbal_serial"] = fw.get("serial")
                data["payload"] = dict(fw)
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
                elif kind == "state":
                    socketio.emit("state", payload)
        except Exception as e:
            debug_log.append(f"emit_worker error: {e}")


def run_bridge(port: str, baud: int = 921600, existing_serial: Optional[serial.Serial] = None) -> Tuple[bool, Optional[str]]:
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
    ok, err_msg = bridge.start(existing_serial=existing_serial)
    if ok:
        _state["connected"] = True
        _state["port"] = port
        _state["fw_info"] = None
        _state["ping_sent_at"] = None
        try:
            _emit_queue.put_nowait(("state", dict(_state)))
        except Full:
            pass
        bridge.send_command(next_seq(), CMD_GET_FW_INFO, b"")
        return True, None
    _state["connected"] = False
    _state["port"] = ""
    _state["gimbal_serial"] = None
    try:
        _emit_queue.put_nowait(("state", dict(_state)))
    except Full:
        pass
    return False, err_msg


@app.route("/api/serial_ports")
def api_serial_ports():
    """List available serial port names (e.g. COM9)."""
    try:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        return {"ok": True, "ports": sorted(ports)}
    except Exception as e:
        return {"ok": False, "error": str(e), "ports": []}


@app.route("/api/rescan_serial", methods=["POST"])
def api_rescan_serial():
    """Request a rescan for gimbal on COM ports (if currently disconnected)."""
    if _state.get("connected"):
        return {"ok": True, "message": "Already connected", "port": _state.get("port")}
    _scan_requested.set()
    return {"ok": True, "message": "Rescan requested"}


@app.route("/api/fw_info")
def api_fw_info():
    """Return current FW info from device (version, serial, slots). Request is sent on connect; call Get FW Info (610) to refresh."""
    fw = _state.get("fw_info")
    if not fw:
        return {"ok": True, "fw_info": None, "message": "Not yet received. Connect serial and use Get FW Info (610) to fetch."}
    va = (fw.get("version_a") or "").strip() or None
    vb = (fw.get("version_b") or "").strip() or None
    active = fw.get("active_slot", 0)
    version = va if active == 0 else vb
    return {
        "ok": True,
        "fw_info": {
            "version": version,
            "version_a": va,
            "version_b": vb,
            "active_slot": active,
            "serial": fw.get("serial"),
            "model_id": fw.get("model_id"),
        },
    }


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
        _state["gimbal_serial"] = None
        _state["fw_info"] = None
        _state["ping_sent_at"] = None
        try:
            _emit_queue.put_nowait(("state", dict(_state)))
        except Full:
            pass
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
    if "--camera" in sys.argv:
        i = sys.argv.index("--camera")
        if i + 1 < len(sys.argv):
            CAMERA_INDEX = int(sys.argv[i + 1])
    else:
        CAMERA_INDEX = int(os.environ.get("CAMERA_INDEX", "0"))
    emit_thread = threading.Thread(target=_emit_worker, daemon=True)
    emit_thread.start()
    auto_connect_thread = threading.Thread(target=_auto_connect_worker, daemon=True)
    auto_connect_thread.start()
    keepalive_thread = threading.Thread(target=_keepalive_worker, daemon=True)
    keepalive_thread.start()
    if CAMERA_AVAILABLE:
        tag_thread = threading.Thread(target=tag_emitter, daemon=True)
        tag_thread.start()
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)

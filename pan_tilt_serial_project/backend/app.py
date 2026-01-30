"""
PC backend: serial bridge to ESP32 (binary protocol), WebSocket push to web GUI.
Debug: last N log lines and connection status in debug panel.
"""

import os
import threading
import time
from flask import Flask, send_from_directory
from flask_socketio import SocketIO, emit

from typing import Optional
from serial_bridge import DebugLog, SerialBridge
from protocol import (
    RSP_ACK_EXECUTED,
    decode_imu,
    decode_ina,
    decode_move_feedback,
)

app = Flask(__name__, static_folder="static", static_url_path="")
app.config["SECRET_KEY"] = "gimbal-backend"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

debug_log = DebugLog(max_lines=500)
bridge: Optional[SerialBridge] = None
_seq = 0
_state = {"connected": False, "port": "", "last_move": None, "last_imu": None, "last_ina": None}


def next_seq() -> int:
    global _seq
    _seq = (_seq + 1) & 0xFFFF
    return _seq


def on_frame(seq: int, type_id: int, payload: bytes) -> None:
    data = {"seq": seq, "type": type_id}
    if type_id == RSP_ACK_EXECUTED and len(payload) >= 8:
        fb = decode_move_feedback(payload)
        if fb:
            _state["last_move"] = fb
            data["payload"] = fb
    elif type_id == 1002 and len(payload) >= 50:
        imu = decode_imu(payload)
        if imu:
            _state["last_imu"] = imu
            data["payload"] = imu
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
    with app.app_context():
        socketio.emit("frame", data)


def _on_log(msg: str) -> None:
    ts = time.strftime("%H:%M:%S", time.localtime())
    line = f"[{ts}] {msg}"
    with app.app_context():
        socketio.emit("log_line", {"line": line})


def run_bridge(port: str, baud: int = 921600) -> bool:
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
    if bridge.start():
        _state["connected"] = True
        _state["port"] = port
        socketio.emit("state", _state)
        return True
    _state["connected"] = False
    _state["port"] = ""
    socketio.emit("state", _state)
    return False


@app.route("/")
def index():
    return send_from_directory("static", "index.html")


@socketio.on("connect")
def on_connect():
    emit("state", _state)
    emit("log_history", {"lines": debug_log.get_recent(100)})


@socketio.on("connect_serial")
def on_connect_serial(data):
    port = data.get("port", "COM3")
    baud = int(data.get("baud", 921600))
    ok = run_bridge(port, baud)
    emit("connect_serial_result", {"ok": ok, "port": port})


@socketio.on("disconnect_serial")
def on_disconnect_serial():
    global bridge
    if bridge:
        bridge.stop()
        bridge = None
    _state["connected"] = False
    _state["port"] = ""
    socketio.emit("state", _state)


@socketio.on("command")
def on_command(data):
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
    if not bridge or not bridge.send_command(seq, type_id, payload):
        emit("error", {"msg": "Serial not connected or send failed"})


@socketio.on("get_log")
def on_get_log():
    emit("log_history", {"lines": debug_log.get_recent(100)})


if __name__ == "__main__":
    import sys
    port_arg = sys.argv[1] if len(sys.argv) > 1 else None
    if port_arg:
        run_bridge(port_arg)
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)

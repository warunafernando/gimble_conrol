"""
Serial bridge: read binary frames from ESP32, log and broadcast to WebSocket.
Accept commands from WebSocket, encode and send to serial.
Debug: ring buffer of last N log lines for debug panel.
"""

import threading
import time
from collections import deque
from typing import Callable, Optional

import serial
from protocol import (
    RSP_ACK_EXECUTED,
    RSP_ACK_RECEIVED,
    RSP_NACK,
    build_frame,
    decode_imu,
    decode_move_feedback,
    format_rx_for_log,
    format_tx_for_log,
    parse_frame,
)

DEBUG_LOG_MAX = 500


class DebugLog:
    def __init__(self, max_lines: int = DEBUG_LOG_MAX):
        self._lines: deque = deque(maxlen=max_lines)
        self._lock = threading.Lock()

    def append(self, line: str) -> None:
        with self._lock:
            ts = time.strftime("%H:%M:%S", time.localtime())
            self._lines.append(f"[{ts}] {line}")

    def get_recent(self, n: int = 100) -> list:
        with self._lock:
            return list(self._lines)[-n:]


def find_frame_start(data: bytes) -> int:
    try:
        return data.index(0x02)
    except ValueError:
        return -1


class SerialBridge:
    def __init__(
        self,
        port: str,
        baud: int = 921600,
        on_frame: Optional[Callable[[int, int, bytes], None]] = None,
        on_log: Optional[Callable[[str], None]] = None,
        debug_log: Optional[DebugLog] = None,
    ):
        self.port = port
        self.baud = baud
        self.on_frame = on_frame
        self.on_log = on_log
        self.log = debug_log or DebugLog()
        self._serial: Optional[serial.Serial] = None
        self._rx_buf = bytearray()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def _log(self, msg: str) -> None:
        self.log.append(msg)
        if self.on_log:
            self.on_log(msg)

    def start(self) -> bool:
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.01)
            self._log(f"Serial opened {self.port} @ {self.baud}")
        except Exception as e:
            self._log(f"Serial open failed: {e}")
            return False
        self._running = True
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
        self._log("Serial closed")

    def send(self, frame: bytes, skip_log: bool = False) -> bool:
        if not self._serial or not self._serial.is_open:
            return False
        try:
            self._serial.write(frame)
            if not skip_log:
                self._log(f"TX {len(frame)} bytes")
            return True
        except Exception as e:
            self._log(f"TX error: {e}")
            return False

    def send_command(self, seq: int, type_id: int, payload: bytes = b"") -> bool:
        try:
            payload = payload or b""
            self._log(format_tx_for_log(seq, type_id, payload))
            frame = build_frame(seq, type_id, payload if payload else None)
            return self.send(frame, skip_log=True)
        except Exception as e:
            self._log(f"Build frame error: {e}")
            return False

    def _read_loop(self) -> None:
        while self._running and self._serial and self._serial.is_open:
            try:
                chunk = self._serial.read(256)
                if chunk:
                    self._rx_buf.extend(chunk)
                    self._process_rx()
            except Exception as e:
                self._log(f"RX error: {e}")
                break
            time.sleep(0.001)
        self._running = False

    def _process_rx(self) -> None:
        while len(self._rx_buf) >= 8:
            idx = find_frame_start(self._rx_buf)
            if idx < 0:
                self._rx_buf.clear()
                return
            if idx > 0:
                del self._rx_buf[:idx]
            length = self._rx_buf[1]
            if length < 4 or length > 251:
                self._rx_buf.pop(0)
                continue
            frame_size = 4 + length
            if len(self._rx_buf) < frame_size:
                return
            frame_bytes = bytes(self._rx_buf[:frame_size])
            result = parse_frame(frame_bytes)
            del self._rx_buf[:frame_size]
            if result is None:
                self._log("RX bad frame (CRC or format)")
                continue
            seq, type_id, payload = result
            self._log(format_rx_for_log(seq, type_id, payload))
            if self.on_frame:
                self.on_frame(seq, type_id, payload)
        if len(self._rx_buf) > 4096:
            self._rx_buf.clear()
            self._log("RX buffer overflow, cleared")

"""Servo (turntable firmware) serial client + streaming protocol parser.

Protocol (115200 8N1, commands start with '#', end with '!'):
    MOVE : #000P{pos}T{time}!        -> no reply (non-blocking)
    READ : #000PRAD!                  -> #000P{pos}!
    STOP : #000PDST!                  -> #OK!
    RST  : #000PRST!                  -> #OK!  (blocking on firmware until homed)

The parser tolerates the firmware's debug strings (``BOOT:``/``DBG:``/``MOV:``)
and partial / interleaved packets.
"""

from __future__ import annotations

import queue
import re
import threading
import time
from typing import Callable, List, Optional, Tuple

try:
    import serial  # pyserial

    _PYSERIAL_AVAILABLE = True
except Exception:  # pragma: no cover
    _PYSERIAL_AVAILABLE = False


# --- protocol parser (Task 4) ----------------------------------------------
_OK_EVENT = ("ok",)
_POSITION_RE = re.compile(rb"^(\d{3})P(\d+)$")


class ProtocolParser:
    def __init__(self, servo_id: int = 0):
        self._id = servo_id
        self._buf = bytearray()
        self._id_bytes = f"{servo_id:03d}".encode()

    def feed(self, data: bytes) -> List[tuple]:
        self._buf.extend(data)
        events: List[tuple] = []
        while True:
            start = self._buf.find(b"#")
            if start < 0:
                self._buf.clear()
                break
            if start > 0:
                del self._buf[:start]
            end = self._buf.find(b"!")
            if end < 0:
                break
            chunk = bytes(self._buf[1:end])
            del self._buf[: end + 1]
            if chunk == b"OK":
                events.append(_OK_EVENT)
            else:
                m = _POSITION_RE.match(chunk)
                if m and m.group(1) == self._id_bytes:
                    events.append(("pos", int(m.group(2))))
        return events


# --- servo errors + poll helper (Task 5) -----------------------------------
class ServoError(RuntimeError):
    pass


def poll_until_reached(
    read_position: Callable[[float], int],
    deg_of: Callable[[int], float],
    pos_target: int,
    tol_deg: float,
    stable_count: int,
    timeout_s: float,
    poll_hz: float,
    progress_cb: Optional[Callable[[float], None]] = None,
) -> float:
    period = 1.0 / poll_hz
    start_deg = None
    count = 0
    start_t = time.monotonic()
    n = 0
    while True:
        t0 = time.monotonic()
        if t0 - start_t > timeout_s:
            raise ServoError("position timeout")
        pos = read_position(timeout_s=period * 2.0)
        deg = deg_of(pos)
        if start_deg is None:
            start_deg = deg
        if abs(deg - deg_of(pos_target)) <= tol_deg:
            count += 1
        else:
            count = 0
        n += 1
        if progress_cb and n % 10 == 0:
            total = abs(deg_of(pos_target) - start_deg)
            remain = max(0.0, abs(deg_of(pos_target) - deg))
            pct = 0.0 if total <= 0 else 1.0 - remain / total
            progress_cb(min(0.999, pct))
        if count >= stable_count:
            if progress_cb:
                progress_cb(1.0)
            return deg
        elapsed = time.monotonic() - t0
        if elapsed < period:
            time.sleep(period - elapsed)


# --- serial client ----------------------------------------------------------
class ServoClient:
    def __init__(
        self,
        port: str,
        baud: int = 115200,
        servo_id: int = 0,
        pos_origin: int = 500,
        deg_per_pos: float = 0.02,
        serial_factory=None,
    ):
        self._port = port
        self._baud = baud
        self._servo_id = servo_id
        self._origin = pos_origin
        self._dpp = deg_per_pos
        self._serial_factory = serial_factory
        self._ser = None
        self._parser = ProtocolParser(servo_id)
        self._reply_q: "queue.Queue[tuple]" = queue.SimpleQueue()
        self._write_lock = threading.Lock()
        self._reader_thread: Optional[threading.Thread] = None
        self._running = False

    def open(self):
        if not self._port:
            raise ServoError("serial_port not configured")
        factory = self._serial_factory
        if factory is None:
            if not _PYSERIAL_AVAILABLE:
                raise ServoError("pyserial (python3-serial) not available")
            factory = serial.Serial
        self._ser = factory(self._port, self._baud, timeout=0.05)
        self._running = True
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

    def close(self):
        self._running = False
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=0.5)

    def _read_loop(self):
        while self._running:
            try:
                data = self._ser.read(256)
            except Exception:
                break
            if data:
                for ev in self._parser.feed(data):
                    self._reply_q.put(ev)

    def _send(self, payload: bytes):
        with self._write_lock:
            if self._ser is None:
                raise ServoError("serial not open")
            self._ser.write(payload)

    def _wait_event(self, kinds: tuple, timeout_s: float):
        deadline = time.monotonic() + timeout_s
        while True:
            remain = deadline - time.monotonic()
            if remain <= 0:
                raise ServoError(f"timeout waiting for {kinds}")
            try:
                ev = self._reply_q.get(timeout=min(remain, 0.1))
            except queue.Empty:
                continue
            if ev[0] in kinds:
                return ev

    def move_to(self, pos: int, time_ms: int):
        self._send(f"#{self._servo_id:03d}P{pos}T{time_ms}!".encode())

    def stop(self):
        self._send(f"#{self._servo_id:03d}PDST!".encode())
        self._wait_event(("ok",), 0.5)

    def read_position(self, timeout_s: float = 0.2) -> int:
        self._send(f"#{self._servo_id:03d}PRAD!".encode())
        ev = self._wait_event(("pos",), timeout_s)
        return int(ev[1])

    def reset(self, timeout_s: float = 30.0):
        self._send(f"#{self._servo_id:03d}PRST!".encode())
        self._wait_event(("ok",), timeout_s)

    def pos_to_deg(self, pos: int) -> float:
        return (pos - self._origin) * self._dpp

    def deg_to_pos(self, deg: float) -> int:
        return int(round(self._origin + deg / self._dpp))

    def poll_until_reached(
        self,
        pos_target: int,
        tol_deg: float,
        stable_count: int = 5,
        timeout_s: float = 30.0,
        poll_hz: float = 100.0,
        progress_cb=None,
    ) -> float:
        return poll_until_reached(
            self.read_position,
            self.pos_to_deg,
            pos_target,
            tol_deg,
            stable_count,
            timeout_s,
            poll_hz,
            progress_cb,
        )

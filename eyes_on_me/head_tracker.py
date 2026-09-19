"""UDP receiver for the sony-head-tracker JSON telemetry stream.

The bridge (``sony-head-tracker-macos bridge`` or the SwiftUI app) emits one
JSON object per sample on ``127.0.0.1:<port+1>`` (default 4243). See
``sony-head-tracker/docs/PROTOCOL.md``. Only ``yprDegrees`` and
``resetCounter`` are consumed here; unknown keys are ignored per the protocol.
"""

from __future__ import annotations

import json
import socket
import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class HeadSample:
    yaw: float  # degrees, tracker convention
    pitch: float
    roll: float
    reset_counter: int
    received_at: float  # time.monotonic()


class HeadTrackerReceiver:
    """Background thread that keeps only the most recent head sample.

    The control loop polls :meth:`latest`; older datagrams are dropped so a
    slow consumer never lags behind the headset.
    """

    def __init__(self, host: str = "127.0.0.1", port: int = 4243, stale_after_s: float = 0.5):
        self._addr = (host, port)
        self._stale_after_s = stale_after_s
        self._lock = threading.Lock()
        self._latest: HeadSample | None = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="head-tracker-udp", daemon=True)
        self._sock: socket.socket | None = None
        self.packets = 0
        self.decode_errors = 0

    def start(self) -> "HeadTrackerReceiver":
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(self._addr)
        self._sock.settimeout(0.2)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()

    def latest(self) -> HeadSample | None:
        """Most recent sample, or ``None`` if nothing fresh has arrived."""
        with self._lock:
            s = self._latest
        if s is None or time.monotonic() - s.received_at > self._stale_after_s:
            return None
        return s

    def _run(self) -> None:
        assert self._sock is not None
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            sample = parse_sample(data)
            if sample is None:
                self.decode_errors += 1
                continue
            self.packets += 1
            with self._lock:
                self._latest = sample


def parse_sample(data: bytes, now: float | None = None) -> HeadSample | None:
    try:
        obj = json.loads(data)
        yaw, pitch, roll = (float(v) for v in obj["yprDegrees"])
        return HeadSample(
            yaw=yaw,
            pitch=pitch,
            roll=roll,
            reset_counter=int(obj.get("resetCounter", 0)),
            received_at=time.monotonic() if now is None else now,
        )
    except (ValueError, KeyError, TypeError):
        return None

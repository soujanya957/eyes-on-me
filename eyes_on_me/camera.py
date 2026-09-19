"""Live view of one Spot camera, selectable by hand or by head yaw.

Fetching runs in a background thread (each RPC is ~50-100 ms); drawing must
happen on the main thread because of macOS/OpenCV, so the control loop calls
:meth:`CameraViewer.pump` once per tick.
"""

from __future__ import annotations

import logging
import threading
import time

import cv2
import numpy as np
from bosdyn.api import image_pb2
from bosdyn.client.image import ImageClient, build_image_request

log = logging.getLogger("eyes_on_me.camera")

# Short name -> Spot image source. Front is split into two fisheyes that face
# slightly outward; ``auto`` picks the one on the side you're looking toward.
SOURCES = {
    "front": "frontleft_fisheye_image",
    "front-right": "frontright_fisheye_image",
    "left": "left_fisheye_image",
    "right": "right_fisheye_image",
    "back": "back_fisheye_image",
    "hand": "hand_color_image",
}
HOTKEYS = {ord("1"): "front", ord("2"): "front-right", ord("3"): "left", ord("4"): "right", ord("5"): "back", ord("6"): "hand"}
# Body cameras are mounted at odd angles; these upright them (from the SDK's get_image example).
ROTATION_DEG = {
    "frontleft_fisheye_image": -78,
    "frontright_fisheye_image": -102,
    "left_fisheye_image": 0,
    "right_fisheye_image": 180,
    "back_fisheye_image": 0,
    "hand_color_image": 0,
}
WINDOW = "eyes-on-me"


def camera_for_yaw(yaw_deg: float) -> str:
    """Which body camera looks roughly where the head is pointing (+yaw = left)."""
    if yaw_deg > 110 or yaw_deg < -110:
        return "back"
    if yaw_deg > 35:
        return "left"
    if yaw_deg < -35:
        return "right"
    return "front" if yaw_deg >= 0 else "front-right"


def decode(img: image_pb2.ImageResponse) -> np.ndarray | None:
    shot = img.shot.image
    buf = np.frombuffer(shot.data, dtype=np.uint8)
    if shot.format == image_pb2.Image.FORMAT_RAW:
        if shot.pixel_format == image_pb2.Image.PIXEL_FORMAT_GREYSCALE_U8:
            frame = buf.reshape(shot.rows, shot.cols)
        elif shot.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
            frame = cv2.cvtColor(buf.reshape(shot.rows, shot.cols, 3), cv2.COLOR_RGB2BGR)
        else:
            return None
    else:
        frame = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
    if frame is None:
        return None
    angle = ROTATION_DEG.get(img.source.name, 0)
    if angle:
        h, w = frame.shape[:2]
        m = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
        frame = cv2.warpAffine(frame, m, (w, h))
    return frame


class CameraViewer:
    def __init__(self, image_client: ImageClient, selection: str, fps: float = 10.0):
        self._client = image_client
        self.selection = selection  # "auto" or a key of SOURCES
        self._auto_choice = "front"
        self._fps = fps
        self._lock = threading.Lock()
        self._frame: np.ndarray | None = None
        self._frame_source = ""
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True, name="camera-fetch")
        self.available = {s.name for s in image_client.list_image_sources()}

    def start(self) -> "CameraViewer":
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()
        cv2.destroyAllWindows()

    @property
    def current(self) -> str:
        return self._auto_choice if self.selection == "auto" else self.selection

    def update_auto(self, head_yaw_deg: float) -> None:
        self._auto_choice = camera_for_yaw(head_yaw_deg)

    def pump(self) -> str | None:
        """Draw the latest frame; return a key event ('recenter', 'quit') if any."""
        with self._lock:
            frame, src = self._frame, self._frame_source
        if frame is not None:
            frame = frame.copy()
            cv2.putText(frame, src, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.imshow(WINDOW, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            return "quit"
        if key == ord("r"):
            return "recenter"
        if key == ord("0"):
            self.selection = "auto"
        elif key in HOTKEYS:
            self.selection = HOTKEYS[key]
        return None

    def _run(self) -> None:
        period = 1.0 / self._fps
        while not self._stop.is_set():
            t0 = time.monotonic()
            name = SOURCES[self.current]
            if name not in self.available:
                with self._lock:
                    self._frame, self._frame_source = None, f"{name} (not on this robot)"
                time.sleep(period)
                continue
            try:
                resp = self._client.get_image([build_image_request(name, quality_percent=60)])
                frame = decode(resp[0])
                if frame is not None:
                    with self._lock:
                        self._frame, self._frame_source = frame, name
            except Exception as e:  # keep the loop alive across transient RPC errors
                log.warning("image fetch failed: %s", e)
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

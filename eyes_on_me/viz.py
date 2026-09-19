"""Headphones-only visualiser: a 3D head gizmo driven by the tracker stream.

Nothing here touches Spot. It draws the *mapped* orientation (after recenter,
sign flips, deadband and smoothing) so what you see is what the robot would
be told to do, and the raw tracker numbers alongside for comparison.
"""

from __future__ import annotations

import math
import time

import cv2
import numpy as np

from .gaze_mapper import GazeConfig, GazeMapper
from .head_tracker import HeadTrackerReceiver

W, H = 720, 560
WINDOW = "eyes-on-me viz"


def rot_zyx(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """Spot body convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll), radians."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def _view_basis(azimuth_deg: float, elevation_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Screen-right and screen-up vectors for an orthographic camera at the given direction."""
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    cam = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    fwd = -cam
    right = np.cross(fwd, [0, 0, 1]); right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return right, up


# Fixed camera: in front of the head, off to its right (-y) and above, so the
# nose points toward the viewer and yaw/pitch/roll all read clearly.
_RIGHT, _UP = _view_basis(azimuth_deg=-40, elevation_deg=22)


def project(p: np.ndarray, centre: tuple[int, int], scale: float) -> tuple[int, int]:
    return int(centre[0] + scale * float(p @ _RIGHT)), int(centre[1] - scale * float(p @ _UP))


# Wireframe head: a box with a nose along +x (forward), "ears" along ±y.
_BOX = np.array(
    [[1, 1, 1], [1, -1, 1], [-1, -1, 1], [-1, 1, 1], [1, 1, -1], [1, -1, -1], [-1, -1, -1], [-1, 1, -1]], float
) * [0.8, 0.9, 1.0]
_EDGES = [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]
_NOSE = np.array([[0.8, 0, 0.1], [1.4, 0, -0.1]])
_EARS = [np.array([[0, 0.9, 0], [0, 1.2, 0]]), np.array([[0, -0.9, 0], [0, -1.2, 0]])]
_EYES = [np.array([0.8, 0.35, 0.35]), np.array([0.8, -0.35, 0.35])]


def draw_gizmo(img: np.ndarray, r: np.ndarray, centre: tuple[int, int], scale: float) -> None:
    p = lambda v: project(r @ v, centre, scale)  # noqa: E731
    # Ground reference frame (unrotated) in grey so the rotation reads clearly.
    o = project(np.zeros(3), centre, scale)
    for axis, colour in zip(np.eye(3) * 1.8, [(90, 90, 90)] * 3):
        cv2.line(img, o, project(axis, centre, scale), colour, 1, cv2.LINE_AA)
    for a, b in _EDGES:
        cv2.line(img, p(_BOX[a]), p(_BOX[b]), (200, 200, 200), 1, cv2.LINE_AA)
    cv2.line(img, p(_NOSE[0]), p(_NOSE[1]), (0, 200, 255), 3, cv2.LINE_AA)
    for ear in _EARS:
        cv2.line(img, p(ear[0]), p(ear[1]), (200, 200, 200), 2, cv2.LINE_AA)
    for eye in _EYES:
        cv2.circle(img, p(eye), 5, (255, 255, 255), -1, cv2.LINE_AA)
    # Body axes: x red (forward), y green (left), z blue (up).
    for axis, colour in zip(np.eye(3) * 1.6, [(0, 0, 255), (0, 255, 0), (255, 80, 0)]):
        cv2.arrowedLine(img, o, p(axis), colour, 2, cv2.LINE_AA, tipLength=0.12)


def bar(img: np.ndarray, x: int, y: int, value: float, limit: float, label: str, colour) -> None:
    """Horizontal bar of ``value`` within ±``limit``; turns red when clamped."""
    w = 220
    cv2.rectangle(img, (x, y), (x + w, y + 14), (60, 60, 60), 1)
    cv2.line(img, (x + w // 2, y), (x + w // 2, y + 14), (90, 90, 90), 1)
    frac = max(-1.0, min(1.0, value / limit)) if limit else 0.0
    end = int(x + w // 2 + frac * w / 2)
    col = (0, 0, 255) if abs(value) >= limit - 1e-6 else colour
    cv2.rectangle(img, (x + w // 2, y + 2), (end, y + 12), col, -1)
    cv2.putText(img, f"{label} {value:6.1f} / ±{limit:.0f}", (x + w + 10, y + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (220, 220, 220), 1)


def main(port: int, cfg: GazeConfig, arm: bool = False) -> int:
    rx = HeadTrackerReceiver(port=port).start()
    mapper = GazeMapper(cfg)
    need_recenter = True
    cv2.namedWindow(WINDOW)
    last_pk, last_t, pps = 0, time.monotonic(), 0.0
    print("viz: r = recenter, a = toggle body/arm envelope, q = quit")
    try:
        while True:
            img = np.full((H, W, 3), 18, np.uint8)
            s = rx.latest()
            now = time.monotonic()
            if now - last_t >= 1.0:
                pps, last_pk, last_t = (rx.packets - last_pk) / (now - last_t), rx.packets, now
            if s is None:
                cv2.putText(img, f"waiting for tracker on 127.0.0.1:{port} ...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.putText(img, "start it with ./scripts/run-tracker.sh", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                draw_gizmo(img, np.eye(3), (W // 2, H // 2 + 20), 90)
            else:
                if need_recenter:
                    mapper.recenter(s.yaw, s.pitch, s.roll)
                    need_recenter = False
                t = mapper.arm_target(s.yaw, s.pitch, s.roll) if arm else mapper.pose_target(s.yaw, s.pitch, s.roll)
                draw_gizmo(img, rot_zyx(t.yaw_rad, t.pitch_rad, t.roll_rad), (W // 2, H // 2 + 20), 90)
                cv2.putText(img, f"{'WH-1000XM5' if s else ''}  {pps:4.1f} pkt/s   resets={s.reset_counter}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 160, 160), 1)
                cv2.putText(img, f"raw   yaw {s.yaw:7.1f}  pitch {s.pitch:7.1f}  roll {s.roll:7.1f}", (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
                cv2.putText(img, f"spot  yaw {t.yaw_deg:7.1f}  pitch {t.pitch_deg:7.1f}  roll {t.roll_deg:7.1f}   ({'arm' if arm else 'body'} envelope)", (20, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 120), 1)
                c = cfg
                lim = (c.max_arm_yaw_deg, c.max_arm_pitch_deg, c.max_arm_roll_deg) if arm else (c.max_body_yaw_deg, c.max_body_pitch_deg, c.max_body_roll_deg)
                bar(img, 20, H - 90, t.yaw_deg, lim[0], "yaw  ", (0, 255, 0))
                bar(img, 20, H - 62, t.pitch_deg, lim[1], "pitch", (255, 80, 0))
                bar(img, 20, H - 34, t.roll_deg, lim[2], "roll ", (0, 0, 255))
            cv2.putText(img, "r recenter   a body/arm   q quit", (W - 300, H - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 120, 120), 1)
            cv2.imshow(WINDOW, img)
            key = cv2.waitKey(16) & 0xFF
            if key == ord("q") or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("r"):
                need_recenter = True
            if key == ord("a"):
                arm = not arm
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()
        cv2.destroyAllWindows()
    return 0

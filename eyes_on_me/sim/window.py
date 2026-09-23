"""The sim's window: a chase camera on Spot plus an inset of what Spot sees.

Rendered offscreen with :class:`mujoco.Renderer` and shown with OpenCV, like
the viz and camera windows, so it runs on the control loop's main thread
without ``mjpython``. Keys match the viz window, plus camera controls.
"""

from __future__ import annotations

import math

import cv2
import mujoco
import numpy as np

from .spot_sim import SimSpot

WINDOW = "eyes-on-me sim"
W, H = 960, 640
INSET_W, INSET_H = 320, 240


def _look_camera(pos: np.ndarray, rot: np.ndarray, distance: float = 1.0) -> mujoco.MjvCamera:
    """A free camera at ``pos`` looking along ``rot``'s x axis (MuJoCo cameras have no roll)."""
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    fwd = rot[:, 0]
    cam.lookat[:] = pos + fwd * distance
    cam.distance = distance
    cam.azimuth = math.degrees(math.atan2(fwd[1], fwd[0]))
    cam.elevation = math.degrees(math.asin(float(np.clip(fwd[2], -1.0, 1.0))))
    return cam


class SimWindow:
    KEYS = "s start/stop  w stand/walk  r recenter  Space E-STOP  Esc E-STOP NOW  q quit  |  [ ] orbit  - = zoom  c chase"

    def __init__(self, sim: SimSpot):
        self.sim = sim
        self._main = mujoco.Renderer(sim.model, H, W)
        self._inset = mujoco.Renderer(sim.model, INSET_H, INSET_W)
        self._cam = mujoco.MjvCamera()
        self._cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self._cam.trackbodyid = sim.body_id
        self._cam.distance = 3.2
        self._cam.elevation = -18.0
        self._orbit = 0.0      # degrees added to the chase azimuth
        self._chase = True     # camera swings round behind Spot as it turns
        cv2.namedWindow(WINDOW)

    def pump(self, status: tuple[str, tuple[int, int, int]] | None) -> str | None:
        """Draw a frame; return a key event like the viz window, or None."""
        sim = self.sim
        self._cam.azimuth = (math.degrees(sim.yaw) if self._chase else 0.0) + self._orbit
        self._main.update_scene(sim.data, self._cam)
        img = cv2.cvtColor(self._main.render(), cv2.COLOR_RGB2BGR)

        pos, rot = sim.hand_pose() if sim.arm_out else sim.front_pose()
        if sim.arm_out:
            pos = pos + rot[:, 0] * 0.22  # past the jaws
        self._inset.update_scene(sim.data, _look_camera(pos, rot))
        inset = cv2.cvtColor(self._inset.render(), cv2.COLOR_RGB2BGR)
        x0, y0 = W - INSET_W - 12, 12
        img[y0:y0 + INSET_H, x0:x0 + INSET_W] = inset
        cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + INSET_W, y0 + INSET_H), (200, 200, 200), 1)
        cv2.putText(img, "hand camera" if sim.arm_out else "front", (x0 + 6, y0 + INSET_H - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        cv2.putText(img, "SIM", (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
        if status is not None:
            text, colour = status
            cv2.putText(img, text, (80, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2)
        if sim.estopped:
            cv2.putText(img, "E-STOPPED", (16, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        x, y, yaw = sim.footprint
        cv2.putText(img, f"x {x:5.2f}  y {y:5.2f}  heading {math.degrees(yaw):6.1f}", (16, H - 34),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
        cv2.putText(img, self.KEYS, (16, H - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1)
        cv2.imshow(WINDOW, img)

        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            return "estop_now"
        if key == ord(" "):
            return "estop"
        events = {ord("q"): "quit", ord("r"): "recenter", ord("s"): "toggle", ord("w"): "walk"}
        if key in events:
            return events[key]
        if key == ord("["):
            self._orbit -= 15.0
        elif key == ord("]"):
            self._orbit += 15.0
        elif key == ord("-"):
            self._cam.distance = min(12.0, self._cam.distance * 1.15)
        elif key == ord("="):
            self._cam.distance = max(1.0, self._cam.distance / 1.15)
        elif key == ord("c"):
            self._chase = not self._chase
        return None

    def close(self) -> None:
        cv2.destroyWindow(WINDOW)
        self._main.close()
        self._inset.close()

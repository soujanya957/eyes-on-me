"""Pure math: head orientation (degrees) -> Spot body pose + turn rate.

No SDK imports here so it is unit-testable without a robot.

Conventions
-----------
* Input yaw/pitch/roll are whatever the head tracker emits, in degrees. The
  tracker's default axis convention makes yaw positive when you look *left*
  and pitch positive when you look *up* (OpenTrack style). Use the ``invert_*``
  flags if your build differs; ``--dry-run`` prints the mapped values so you
  can check before the robot moves.
* Output follows Spot's right-handed body frame: +yaw = nose left (CCW from
  above), +pitch = nose *down*, +roll = right side down. Hence pitch is
  inverted by default.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


def wrap_deg(a: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


def clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def apply_deadband(v: float, deadband: float) -> float:
    """Zero inside ``|v| < deadband``; re-zeroed continuous outside it."""
    if abs(v) < deadband:
        return 0.0
    return v - math.copysign(deadband, v)


@dataclass
class GazeConfig:
    # Spot's stand-pose envelope. The controller clamps internally too, but
    # keeping our own limits gives a smooth hand-off to turning-in-place.
    max_body_yaw_deg: float = 25.0
    max_body_pitch_deg: float = 20.0
    max_body_roll_deg: float = 12.0
    # Scale head motion -> body motion (1.0 = one-to-one).
    yaw_gain: float = 1.0
    pitch_gain: float = 1.0
    roll_gain: float = 0.5
    deadband_deg: float = 2.0
    # EMA smoothing factor in [0, 1]; 1.0 disables smoothing.
    smoothing: float = 0.35
    invert_yaw: bool = False
    invert_pitch: bool = True
    invert_roll: bool = False
    # Turn-in-place (``--mode turn``): proportional gain on the yaw the body
    # pose can't cover, and a cap on angular velocity.
    turn_kp: float = 1.5  # (rad/s) per rad of residual yaw
    max_turn_rate_rad_s: float = 0.8
    turn_stop_deg: float = 1.5  # don't chase residuals smaller than this


@dataclass(frozen=True)
class BodyTarget:
    yaw_deg: float
    pitch_deg: float
    roll_deg: float
    v_rot_rad_s: float = 0.0  # non-zero only in turn mode

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)

    @property
    def pitch_rad(self) -> float:
        return math.radians(self.pitch_deg)

    @property
    def roll_rad(self) -> float:
        return math.radians(self.roll_deg)


class GazeMapper:
    """Stateful mapper: recenter offset + smoothing + mode-specific yaw split."""

    def __init__(self, cfg: GazeConfig):
        self.cfg = cfg
        self._offset = (0.0, 0.0, 0.0)
        self._smoothed: tuple[float, float, float] | None = None
        self._last_reset_counter: int | None = None

    def recenter(self, yaw: float, pitch: float, roll: float) -> None:
        """Treat the current head pose as 'straight ahead'."""
        self._offset = (yaw, pitch, roll)
        self._smoothed = None

    def note_reset_counter(self, counter: int) -> bool:
        """Track the headset's own re-zero events; returns True when one occurred."""
        changed = self._last_reset_counter is not None and counter != self._last_reset_counter
        self._last_reset_counter = counter
        if changed:
            self._smoothed = None
        return changed

    def head_to_body(self, yaw: float, pitch: float, roll: float) -> tuple[float, float, float]:
        """Recentred, signed, gained, dead-banded, smoothed head angles (deg).

        Yaw is *not* clamped here: in turn mode the surplus drives rotation.
        """
        c = self.cfg
        y = wrap_deg(yaw - self._offset[0]) * (-1.0 if c.invert_yaw else 1.0) * c.yaw_gain
        p = wrap_deg(pitch - self._offset[1]) * (-1.0 if c.invert_pitch else 1.0) * c.pitch_gain
        r = wrap_deg(roll - self._offset[2]) * (-1.0 if c.invert_roll else 1.0) * c.roll_gain
        y, p, r = (apply_deadband(v, c.deadband_deg) for v in (y, p, r))
        if self._smoothed is None or c.smoothing >= 1.0:
            self._smoothed = (y, p, r)
        else:
            a = c.smoothing
            sy, sp, sr = self._smoothed
            self._smoothed = (sy + a * (y - sy), sp + a * (p - sp), sr + a * (r - sr))
        return self._smoothed

    def pose_target(self, yaw: float, pitch: float, roll: float) -> BodyTarget:
        """``pose`` mode: body-only, everything clamped to the stand envelope."""
        c = self.cfg
        y, p, r = self.head_to_body(yaw, pitch, roll)
        return BodyTarget(
            yaw_deg=clamp(y, -c.max_body_yaw_deg, c.max_body_yaw_deg),
            pitch_deg=clamp(p, -c.max_body_pitch_deg, c.max_body_pitch_deg),
            roll_deg=clamp(r, -c.max_body_roll_deg, c.max_body_roll_deg),
        )

    def turn_target(
        self, yaw: float, pitch: float, roll: float, robot_heading_rel_deg: float
    ) -> BodyTarget:
        """``turn`` mode: total desired yaw is split into body yaw + rotation.

        ``robot_heading_rel_deg`` is how far the robot has already turned
        (odom heading now minus odom heading at recenter). The body pose
        absorbs what it can of the remaining error; the surplus becomes an
        angular velocity so the feet walk round until the body can cover it.
        """
        c = self.cfg
        y, p, r = self.head_to_body(yaw, pitch, roll)
        err = wrap_deg(y - robot_heading_rel_deg)
        body_yaw = clamp(err, -c.max_body_yaw_deg, c.max_body_yaw_deg)
        residual = err - body_yaw
        v_rot = 0.0
        if abs(residual) > c.turn_stop_deg:
            v_rot = clamp(c.turn_kp * math.radians(residual), -c.max_turn_rate_rad_s, c.max_turn_rate_rad_s)
        return BodyTarget(
            yaw_deg=body_yaw,
            pitch_deg=clamp(p, -c.max_body_pitch_deg, c.max_body_pitch_deg),
            roll_deg=clamp(r, -c.max_body_roll_deg, c.max_body_roll_deg),
            v_rot_rad_s=v_rot,
        )

"""Run decoded touchpad actions on Spot.

Kept separate from :mod:`eyes_on_me.touchpad.gestures` so the mapping stays
pure and testable, and separate from :class:`~eyes_on_me.spot_gaze.SpotGaze`
so the SDK calls sit in one readable place.

A step is a *trajectory* command, not a timed velocity burst: you give Spot a
relative goal in its own body frame and it walks there and stops. That makes
every gesture bounded by construction - a stray swipe moves the robot one
step, not "0.4 m/s until something cancels it" - which is what PLAN.md
section 5 rule 1 asks for, with less machinery than the dead-man timer the
original plan needed.

Steering stays with the head. The step goal is expressed in the body frame,
so the direction you look is the direction "forward" means.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME  # noqa: F401  (documents the frame)
from bosdyn.client.robot_command import RobotCommandBuilder
from bosdyn.geometry import EulerZXY

from .gestures import (
    ESTOP,
    GRIPPER_TOGGLE,
    STEP_BACK,
    STEP_FORWARD,
    STEP_LEFT,
    STEP_RIGHT,
    STOP,
    Action,
)

log = logging.getLogger("eyes_on_me.touchpad.actions")


@dataclass
class StepConfig:
    forward_m: float = 0.5
    back_m: float = 0.4
    side_m: float = 0.4
    #: How long Spot is given to complete one step before the command expires.
    #: Also how long the controller treats itself as "stepping", which is what
    #: makes a double tap mean abort rather than gripper.
    timeout_s: float = 3.0


#: (dx, dy) in the body frame: +x forward, +y left.
_STEP_VECTORS = {
    STEP_FORWARD: (1.0, 0.0),
    STEP_BACK: (-1.0, 0.0),
    STEP_LEFT: (0.0, 1.0),
    STEP_RIGHT: (0.0, -1.0),
}


class TouchpadActions:
    """Turns :class:`Action` objects into robot commands.

    Holds no lease or e-stop of its own: it is handed the already-connected
    clients, and delegates e-stop to the owner so there is exactly one path
    that cuts power.
    """

    def __init__(self, cmd_client, state_client, cfg: StepConfig, *,
                 body_height: float = 0.0, estop_cb=None, has_arm: bool = False,
                 dry_run: bool = False):
        self._cmd = cmd_client
        self._state = state_client
        self.cfg = cfg
        self._body_height = body_height
        self._estop_cb = estop_cb
        self._has_arm = has_arm
        self._dry_run = dry_run
        self._step_until = 0.0
        self._gripper_open = False

    @property
    def stepping(self) -> bool:
        """True while a step is still expected to be running."""
        return time.monotonic() < self._step_until

    def run(self, action: Action, body_orientation: EulerZXY | None = None) -> None:
        name = action.name
        if name in _STEP_VECTORS:
            self._step(name, body_orientation)
        elif name == STOP:
            self._stop(body_orientation)
        elif name == GRIPPER_TOGGLE:
            self._gripper()
        elif name == ESTOP:
            log.critical("touchpad: E-STOP (double tap x5)")
            if self._estop_cb is not None:
                self._estop_cb()
        else:  # pragma: no cover - decoder only emits the names above
            log.warning("touchpad: unknown action %s", name)

    # --------------------------------------------------------------- motion
    def _step(self, name: str, orientation: EulerZXY | None) -> None:
        ux, uy = _STEP_VECTORS[name]
        dx = ux * (self.cfg.forward_m if ux > 0 else self.cfg.back_m)
        dy = uy * self.cfg.side_m
        log.info("touchpad: %s  (%.2f, %.2f) m", name, dx, dy)
        self._step_until = time.monotonic() + self.cfg.timeout_s
        if self._dry_run:
            return
        snapshot = self._state.get_robot_state().kinematic_state.transforms_snapshot
        params = RobotCommandBuilder.mobility_params(
            body_height=self._body_height,
            footprint_R_body=orientation or EulerZXY(yaw=0.0, roll=0.0, pitch=0.0),
        )
        cmd = RobotCommandBuilder.synchro_trajectory_command_in_body_frame(
            goal_x_rt_body=dx, goal_y_rt_body=dy, goal_heading_rt_body=0.0,
            frame_tree_snapshot=snapshot, params=params,
        )
        self._cmd.robot_command(cmd, end_time_secs=time.time() + self.cfg.timeout_s)

    def _stop(self, orientation: EulerZXY | None) -> None:
        """Cancel a step by replacing it with a stand, which supersedes it."""
        log.info("touchpad: STOP")
        self._step_until = 0.0
        if self._dry_run:
            return
        self._cmd.robot_command(RobotCommandBuilder.synchro_stand_command(
            body_height=self._body_height,
            footprint_R_body=orientation or EulerZXY(yaw=0.0, roll=0.0, pitch=0.0),
        ))

    # -------------------------------------------------------------- gripper
    def _gripper(self) -> None:
        if not self._has_arm:
            log.warning("touchpad: gripper toggle ignored, this Spot has no arm")
            return
        self._gripper_open = not self._gripper_open
        log.info("touchpad: gripper %s", "open" if self._gripper_open else "close")
        if self._dry_run:
            return
        cmd = (RobotCommandBuilder.claw_gripper_open_command() if self._gripper_open
               else RobotCommandBuilder.claw_gripper_close_command())
        self._cmd.robot_command(cmd)

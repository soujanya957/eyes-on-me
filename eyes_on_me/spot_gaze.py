"""Spot side: lease, e-stop, power, and the gaze control loop."""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass

from bosdyn.client import create_standard_sdk
from bosdyn.client.estop import EstopClient, EstopEndpoint, EstopKeepAlive
from bosdyn.client.frame_helpers import get_odom_tform_body
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.geometry import EulerZXY

from .gaze_mapper import BodyTarget, GazeMapper
from .head_tracker import HeadTrackerReceiver

log = logging.getLogger("eyes_on_me")


@dataclass
class RunOptions:
    mode: str = "pose"  # "pose" | "turn"
    rate_hz: float = 20.0
    body_height: float = 0.0
    dry_run: bool = False
    external_estop: bool = False
    sit_on_exit: bool = True
    recenter_on_start: bool = True


class StdinRecenter:
    """Press ``r`` + Enter in the terminal to re-zero without touching the app."""

    def __init__(self):
        self.requested = threading.Event()
        threading.Thread(target=self._run, daemon=True, name="stdin-recenter").start()

    def _run(self):
        for line in sys.stdin:
            if line.strip().lower() == "r":
                self.requested.set()


class SpotGaze:
    def __init__(self, hostname: str, mapper: GazeMapper, opts: RunOptions):
        self.mapper = mapper
        self.opts = opts
        self.robot = None
        self._estop_keepalive = None
        self._lease_keepalive = None
        self._cmd: RobotCommandClient | None = None
        self._state: RobotStateClient | None = None
        self._heading0 = 0.0
        if not opts.dry_run:
            sdk = create_standard_sdk("eyes-on-me")
            self.robot = sdk.create_robot(hostname)

    # ------------------------------------------------------------------ setup
    def connect(self) -> None:
        if self.opts.dry_run:
            log.info("dry run: not connecting to Spot")
            return
        from bosdyn.client.util import authenticate

        authenticate(self.robot)  # BOSDYN_CLIENT_USERNAME/PASSWORD env or prompt
        self.robot.time_sync.wait_for_sync()
        self._state = self.robot.ensure_client(RobotStateClient.default_service_name)
        self._cmd = self.robot.ensure_client(RobotCommandClient.default_service_name)

        if self.opts.external_estop:
            assert not self.robot.is_estopped(), (
                "Robot is e-stopped and --external-estop was given; clear it from the tablet/estop client."
            )
        else:
            estop_client = self.robot.ensure_client(EstopClient.default_service_name)
            endpoint = EstopEndpoint(estop_client, "eyes-on-me", estop_timeout=9.0)
            endpoint.force_simple_setup()
            self._estop_keepalive = EstopKeepAlive(endpoint)

        lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self._lease_keepalive = LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True)

        log.info("powering on")
        self.robot.power_on(timeout_sec=20)
        assert self.robot.is_powered_on(), "power on failed"
        log.info("standing")
        blocking_stand(self._cmd, timeout_sec=10)
        self._heading0 = self._odom_yaw_deg()

    def shutdown(self) -> None:
        if self.robot is None:
            return
        try:
            if self.opts.sit_on_exit and self.robot.is_powered_on():
                log.info("sitting and powering off")
                self.robot.power_off(cut_immediately=False, timeout_sec=20)
        finally:
            if self._lease_keepalive:
                self._lease_keepalive.shutdown()
            if self._estop_keepalive:
                self._estop_keepalive.shutdown()

    # --------------------------------------------------------------- helpers
    def _odom_yaw_deg(self) -> float:
        snap = self._state.get_robot_state().kinematic_state.transforms_snapshot
        return math.degrees(get_odom_tform_body(snap).rot.to_yaw())

    def _send(self, t: BodyTarget) -> None:
        if self.opts.dry_run:
            return
        orient = EulerZXY(yaw=t.yaw_rad, roll=t.roll_rad, pitch=t.pitch_rad)
        if self.opts.mode == "turn" and t.v_rot_rad_s != 0.0:
            params = RobotCommandBuilder.mobility_params(
                body_height=self.opts.body_height, footprint_R_body=orient
            )
            cmd = RobotCommandBuilder.synchro_velocity_command(v_x=0.0, v_y=0.0, v_rot=t.v_rot_rad_s, params=params)
            # Velocity commands must carry an end time; keep it just past one loop period.
            self._cmd.robot_command(cmd, end_time_secs=time.time() + 2.0 / self.opts.rate_hz + 0.1)
        else:
            cmd = RobotCommandBuilder.synchro_stand_command(
                body_height=self.opts.body_height, footprint_R_body=orient
            )
            self._cmd.robot_command(cmd)

    # ------------------------------------------------------------------ loop
    def run(self, receiver: HeadTrackerReceiver) -> None:
        period = 1.0 / self.opts.rate_hz
        recenter = StdinRecenter()
        need_recenter = self.opts.recenter_on_start
        last_log = 0.0
        log.info("mode=%s  press 'r' + Enter to recenter, Ctrl+C to stop", self.opts.mode)
        while True:
            tick = time.monotonic()
            s = receiver.latest()
            if s is None:
                if tick - last_log > 2.0:
                    log.warning("no head-tracker samples (is the bridge running on port %s?)", receiver._addr[1])
                    last_log = tick
                time.sleep(period)
                continue

            if self.mapper.note_reset_counter(s.reset_counter):
                log.info("headset re-zeroed itself (resetCounter=%d)", s.reset_counter)
            if need_recenter or recenter.requested.is_set():
                recenter.requested.clear()
                need_recenter = False
                self.mapper.recenter(s.yaw, s.pitch, s.roll)
                if not self.opts.dry_run:
                    self._heading0 = self._odom_yaw_deg()
                log.info("recentred at yaw=%.1f pitch=%.1f roll=%.1f", s.yaw, s.pitch, s.roll)

            if self.opts.mode == "turn":
                rel = 0.0 if self.opts.dry_run else (self._odom_yaw_deg() - self._heading0)
                target = self.mapper.turn_target(s.yaw, s.pitch, s.roll, rel)
            else:
                target = self.mapper.pose_target(s.yaw, s.pitch, s.roll)

            self._send(target)

            if self.opts.dry_run or tick - last_log > 0.5:
                last_log = tick
                log.info(
                    "head y=%6.1f p=%6.1f r=%6.1f  ->  body y=%6.1f p=%6.1f r=%6.1f  v_rot=%5.2f",
                    s.yaw, s.pitch, s.roll, target.yaw_deg, target.pitch_deg, target.roll_deg, target.v_rot_rad_s,
                )
            time.sleep(max(0.0, period - (time.monotonic() - tick)))

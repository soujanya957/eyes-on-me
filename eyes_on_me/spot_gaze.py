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
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, get_odom_tform_body
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (
    RobotCommandBuilder,
    RobotCommandClient,
    block_until_arm_arrives,
    blocking_stand,
)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.geometry import EulerZXY

from .camera import CameraViewer
from .gaze_mapper import BodyTarget, GazeMapper
from .head_tracker import HeadTrackerReceiver

log = logging.getLogger("eyes_on_me")


@dataclass
class RunOptions:
    mode: str = "pose"  # "pose" | "turn" | "arm"
    camera: str = "none"  # "none" | "auto" | a key of camera.SOURCES
    rate_hz: float = 20.0
    body_height: float = 0.0
    dry_run: bool = False
    external_estop: bool = False
    sit_on_exit: bool = True
    recenter_on_start: bool = True
    # Where the gripper sits while it looks around (flat_body frame, metres).
    hand_x: float = 0.6
    hand_y: float = 0.0
    hand_z: float = 0.45


class StdinKeys:
    """Terminal hotkeys (each followed by Enter): r recenter, e settle-then-cut, E cut now."""

    def __init__(self):
        self.recenter = threading.Event()
        self.estop = threading.Event()       # settle (sit) then cut motor power
        self.estop_now = threading.Event()   # cut motor power immediately
        threading.Thread(target=self._run, daemon=True, name="stdin-keys").start()

    def _run(self):
        for line in sys.stdin:
            k = line.strip()
            if k.lower() == "r":
                self.recenter.set()
            elif k == "e":
                self.estop.set()
            elif k in ("E", "!"):
                self.estop_now.set()


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
        self._viewer: CameraViewer | None = None
        self.estopped = False
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

        if self.opts.mode == "arm":
            assert self.robot.has_arm(), "--mode arm needs a Spot with an arm"
            log.info("unstowing arm")
            cmd_id = self._cmd.robot_command(RobotCommandBuilder.arm_ready_command())
            block_until_arm_arrives(self._cmd, cmd_id, timeout_sec=10)
            log.info("raising hand to look pose (%.2f, %.2f, %.2f)", self.opts.hand_x, self.opts.hand_y, self.opts.hand_z)
            cmd_id = self._cmd.robot_command(self._arm_cmd(BodyTarget(0.0, 0.0, 0.0), seconds=2.0))
            block_until_arm_arrives(self._cmd, cmd_id, timeout_sec=10)

        if self.opts.camera != "none":
            image_client = self.robot.ensure_client(ImageClient.default_service_name)
            self._viewer = CameraViewer(image_client, self.opts.camera).start()

    def estop(self, immediate: bool) -> None:
        """Software e-stop through our own endpoint (unavailable with --external-estop)."""
        if self._estop_keepalive is None:
            log.error("E-STOP requested but this process holds no e-stop endpoint (--external-estop); use the tablet")
            return
        self.estopped = True
        if immediate:
            log.critical("E-STOP: cutting motor power NOW")
            self._estop_keepalive.stop()
        else:
            log.critical("E-STOP: settling (sit) then cutting motor power")
            self._estop_keepalive.settle_then_cut()

    def shutdown(self) -> None:
        if self.robot is None:
            return
        try:
            if self._viewer:
                self._viewer.stop()
            if self.estopped:
                log.info("e-stopped; leaving motors cut. Clear it from the tablet or rerun.")
                return
            if self.opts.mode == "arm" and self.robot.is_powered_on():
                log.info("stowing arm")
                cmd_id = self._cmd.robot_command(RobotCommandBuilder.arm_stow_command())
                block_until_arm_arrives(self._cmd, cmd_id, timeout_sec=10)
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

    def _arm_cmd(self, t: BodyTarget, seconds: float):
        """Hand held at a fixed spot in front of the body, pointing where the head points."""
        q = EulerZXY(yaw=t.yaw_rad, roll=t.roll_rad, pitch=t.pitch_rad).to_quaternion()
        return RobotCommandBuilder.arm_pose_command(
            self.opts.hand_x, self.opts.hand_y, self.opts.hand_z, q.w, q.x, q.y, q.z,
            GRAV_ALIGNED_BODY_FRAME_NAME, seconds=seconds,
        )

    def _send(self, t: BodyTarget) -> None:
        if self.opts.dry_run:
            return
        if self.opts.mode == "arm":
            # Short horizon so each new head sample re-targets the hand smoothly.
            self._cmd.robot_command(self._arm_cmd(t, seconds=3.0 / self.opts.rate_hz))
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
        keys = StdinKeys()
        need_recenter = self.opts.recenter_on_start
        last_log = 0.0
        log.info("mode=%s camera=%s", self.opts.mode, self.opts.camera)
        log.info("keys (+Enter):  r recenter   e E-STOP (sit, then cut)   E E-STOP NOW   Ctrl+C quit")
        if self._viewer:
            log.info("camera window:  r recenter   Space E-STOP (sit, then cut)   Esc E-STOP NOW   q quit")
        while True:
            tick = time.monotonic()
            if keys.estop_now.is_set():
                self.estop(immediate=True)
                return
            if keys.estop.is_set():
                self.estop(immediate=False)
                return
            s = receiver.latest()
            if s is None:
                if tick - last_log > 2.0:
                    log.warning("no head-tracker samples (is the bridge running on port %s?)", receiver._addr[1])
                    last_log = tick
                time.sleep(period)
                continue

            if self.mapper.note_reset_counter(s.reset_counter):
                log.info("headset re-zeroed itself (resetCounter=%d)", s.reset_counter)
            if need_recenter or keys.recenter.is_set():
                keys.recenter.clear()
                need_recenter = False
                self.mapper.recenter(s.yaw, s.pitch, s.roll)
                if not self.opts.dry_run:
                    self._heading0 = self._odom_yaw_deg()
                log.info("recentred at yaw=%.1f pitch=%.1f roll=%.1f", s.yaw, s.pitch, s.roll)

            if self.opts.mode == "turn":
                rel = 0.0 if self.opts.dry_run else (self._odom_yaw_deg() - self._heading0)
                target = self.mapper.turn_target(s.yaw, s.pitch, s.roll, rel)
            elif self.opts.mode == "arm":
                target = self.mapper.arm_target(s.yaw, s.pitch, s.roll)
            else:
                target = self.mapper.pose_target(s.yaw, s.pitch, s.roll)

            self._send(target)

            if self._viewer:
                self._viewer.update_auto(target.yaw_deg if self.opts.mode != "arm" else 0.0)
                event = self._viewer.pump()
                if event == "recenter":
                    need_recenter = True
                elif event == "estop":
                    self.estop(immediate=False)
                    return
                elif event == "estop_now":
                    self.estop(immediate=True)
                    return
                elif event == "quit":
                    log.info("quit from camera window")
                    return

            if self.opts.dry_run or tick - last_log > 0.5:
                last_log = tick
                log.info(
                    "head y=%6.1f p=%6.1f r=%6.1f  ->  body y=%6.1f p=%6.1f r=%6.1f  v_rot=%5.2f",
                    s.yaw, s.pitch, s.roll, target.yaw_deg, target.pitch_deg, target.roll_deg, target.v_rot_rad_s,
                )
            time.sleep(max(0.0, period - (time.monotonic() - tick)))

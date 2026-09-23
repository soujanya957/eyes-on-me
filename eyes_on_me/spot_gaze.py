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
from .gaze_mapper import BodyTarget, GazeMapper, wrap_deg
from .head_tracker import HeadTrackerReceiver
from .touchpad.gestures import ESTOP, GRIPPER_TOGGLE

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
    viz: bool = False           # draw the head gizmo alongside the control loop
    start_paused: bool = True   # stand still until you press s; recenter works while paused
    touchpad: bool = False      # WH-1000XM5 earcup gestures drive stepping and the gripper
    sim: bool = False           # drive a MuJoCo Spot instead of a robot (eyes_on_me.sim)
    # Where the gripper sits while it looks around (flat_body frame, metres).
    hand_x: float = 0.6
    hand_y: float = 0.0
    hand_z: float = 0.55


class StdinKeys:
    """Terminal hotkeys (each followed by Enter): r recenter, s start/stop, w stand/walk, e settle-then-cut, E cut now."""

    def __init__(self):
        self.recenter = threading.Event()
        self.toggle = threading.Event()      # start/stop following the head
        self.walk = threading.Event()        # switch pose (stand) <-> turn (walk)
        self.estop = threading.Event()       # settle (sit) then cut motor power
        self.estop_now = threading.Event()   # cut motor power immediately
        threading.Thread(target=self._run, daemon=True, name="stdin-keys").start()

    def _run(self):
        for line in sys.stdin:
            k = line.strip()
            if k.lower() == "r":
                self.recenter.set()
            elif k.lower() == "s":
                self.toggle.set()
            elif k.lower() == "w":
                self.walk.set()
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
        self._gizmo = None
        self._touchpad = None
        self._sim = None
        self._sim_win = None
        self.estopped = False
        if not opts.dry_run and not opts.sim:
            sdk = create_standard_sdk("eyes-on-me")
            self.robot = sdk.create_robot(hostname)

    # ------------------------------------------------------------------ setup
    def connect(self) -> None:
        if self.opts.dry_run:
            log.info("dry run: not connecting to Spot")
            return
        if self.opts.sim:
            self._connect_sim()
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
            # Spot refuses to reconfigure the e-stop while motors are on, so registering
            # our own endpoint has to happen before anything powers the robot up. If the
            # tablet left it powered, say so plainly instead of surfacing MotorsOnError.
            assert not self.robot.is_powered_on(), (
                "Spot's motors are already on (the tablet usually did this), and the e-stop "
                "cannot be reconfigured while they are. Either power the motors off from the "
                "tablet and rerun, or pass --external-estop to keep the tablet as the e-stop "
                "(you then lose the s/Space software e-stop in this tool)."
            )
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

    def _connect_sim(self) -> None:
        """Same start-up as the robot - power on, stand, unstow - on the MuJoCo Spot.

        The sim takes the robot's own command and state calls, so it stands in
        for both the command and the state client and nothing downstream knows.
        """
        from .sim import SimSpot
        from .sim.window import SimWindow

        log.info("sim: MuJoCo Spot (kinematic pose + IK, not a physics or walking-controller model)")
        self._sim = SimSpot()
        self._cmd = self._state = self._sim
        self._sim_win = SimWindow(self._sim)
        if self.opts.camera != "none":
            log.info("sim: --camera ignored; the sim window shows Spot's front / hand view itself")
            self.opts.camera = "none"
        log.info("powering on")
        log.info("standing")
        self._sim.power_on()
        self._sim_settle(1.5)
        self._heading0 = self._odom_yaw_deg()
        if self.opts.mode == "arm":
            log.info("unstowing arm")
            self._cmd.robot_command(RobotCommandBuilder.arm_ready_command())
            log.info("raising hand to look pose (%.2f, %.2f, %.2f)", self.opts.hand_x, self.opts.hand_y, self.opts.hand_z)
            self._cmd.robot_command(self._arm_cmd(BodyTarget(0.0, 0.0, 0.0), seconds=2.0))
            self._sim_settle(2.0)

    def _sim_settle(self, seconds: float) -> None:
        """Let the sim play out a blocking step (stand, unstow, sit) on screen."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._sim.step()
            if self._sim_win.pump(None) == "quit":
                return
            time.sleep(0.02)

    def estop(self, immediate: bool) -> None:
        """Software e-stop through our own endpoint (unavailable with --external-estop)."""
        if self._sim is not None:
            self.estopped = True
            log.critical("E-STOP (sim): %s", "cutting motor power NOW" if immediate else "settling (sit) then cutting")
            self._sim.estop(immediate)
            return
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
        if self._gizmo:
            self._gizmo.close()
            self._gizmo = None
        if self._touchpad is not None:
            self._touchpad.stop()   # hands the user's volume back
            self._touchpad = None
        if self._sim is not None:
            if not self.estopped and self.opts.sit_on_exit:
                log.info("sitting and powering off")
                self._sim.power_off()
            self._sim_settle(2.5)   # let the sit / e-stop play out on screen
            self._sim_win.close()
            self._sim = self._sim_win = None
            return
        if self.robot is None:
            return
        try:
            if self._viewer:
                self._viewer.stop()
            if self.estopped:
                log.info("e-stopped; leaving motors cut. Clear it from the tablet or rerun.")
                return
            if self._lease_keepalive is None:
                # connect() failed before taking the lease: the robot is not ours to
                # sit or power off, and trying raises NoSuchLease over the real error.
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

    def _status(self, paused: bool) -> tuple[str, tuple[int, int, int]]:
        """Banner for the viz window: red when paused, green when following, plus stand/walk."""
        stance = {"pose": "  STAND", "turn": "  WALK"}.get(self.opts.mode, "")
        return ("PAUSED" + stance, (0, 0, 255)) if paused else ("FOLLOWING" + stance, (0, 220, 120))

    def _toggle_walk(self) -> None:
        """Switch between standing (pose: lean only) and walking (turn: step round to face you).

        Both directions keep "straight ahead" consistent with where the robot
        actually faces: walking measures turns from here, and standing again
        re-zeroes the head on the heading the robot walked round to.
        """
        if self.opts.mode == "arm":
            log.warning("w ignored: stand/walk switching is for pose/turn, not arm mode")
            return
        now = 0.0 if self.opts.dry_run else self._odom_yaw_deg()
        if self.opts.mode == "turn":
            self.mapper.shift_yaw(wrap_deg(now - self._heading0))
            self.opts.mode = "pose"
            log.info("STANDING - body leans where you look, feet stay put (w to walk)")
        else:
            self.opts.mode = "turn"
            log.info("WALKING - Spot steps round to face where you look (w to stand)")
        self._heading0 = now

    def _pump_viewer(self, target: BodyTarget) -> str | None:
        if self._viewer is None:
            return None
        self._viewer.update_auto(target.yaw_deg if self.opts.mode != "arm" else 0.0)
        return self._viewer.pump()

    def _hold(self) -> None:
        """Square up and stand still: sent once when paused, so the body does not
        keep whatever tilt it had when you stopped following."""
        if self.opts.dry_run:
            return
        if self.opts.mode == "arm":
            self._cmd.robot_command(self._arm_cmd(BodyTarget(0.0, 0.0, 0.0), seconds=1.0))
            return
        self._cmd.robot_command(RobotCommandBuilder.synchro_stand_command(
            body_height=self.opts.body_height, footprint_R_body=EulerZXY(yaw=0.0, roll=0.0, pitch=0.0)
        ))

    # ------------------------------------------------------------------ loop
    def run(self, receiver: HeadTrackerReceiver) -> None:
        period = 1.0 / self.opts.rate_hz
        keys = StdinKeys()
        need_recenter = self.opts.recenter_on_start
        paused = self.opts.start_paused
        last_log = 0.0
        if self.opts.viz:
            from .viz import GizmoWindow

            self._gizmo = GizmoWindow(self.mapper.cfg, receiver._addr[1], arm=self.opts.mode == "arm")
        gizmo = self._gizmo
        log.info("mode=%s camera=%s viz=%s", self.opts.mode, self.opts.camera, self.opts.viz)
        log.info("keys (+Enter):  s start/stop   r recenter   w stand/walk   e E-STOP (sit, then cut)   E E-STOP NOW   Ctrl+C quit")
        if self._viewer:
            log.info("camera window:  r recenter   Space E-STOP (sit, then cut)   Esc E-STOP NOW   q quit")
        if gizmo:
            log.info("viz window:     s start/stop   r recenter   w stand/walk   Space E-STOP   Esc E-STOP NOW   q quit")
        touchpad, tp_actions = None, None
        if self.opts.touchpad:
            from .touchpad import TouchpadSource
            from .touchpad.actions import StepConfig, TouchpadActions

            touchpad = self._touchpad = TouchpadSource().start()
            tp_actions = TouchpadActions(
                self._cmd, self._state, StepConfig(), body_height=self.opts.body_height,
                estop_cb=lambda: self.estop(immediate=False),
                has_arm=bool((self.robot and self.robot.has_arm()) or self._sim),
                dry_run=self.opts.dry_run,
            )
            log.info("touchpad:      swipe fwd/back = step fwd/back   swipe up/down = step left/right")
            log.info("touchpad:      double tap = gripper (or abort a step)   double tap x5 = E-STOP")

        if paused:
            log.info("PAUSED - standing still. Face forward, press r to recenter, then s to start following.")
            self._hold()
        while True:
            tick = time.monotonic()
            if self._sim is not None:
                self._sim.step(tick)
            if keys.estop_now.is_set():
                self.estop(immediate=True)
                return
            if keys.estop.is_set():
                self.estop(immediate=False)
                return
            if keys.walk.is_set():
                keys.walk.clear()
                self._toggle_walk()
            if keys.toggle.is_set():
                keys.toggle.clear()
                paused = not paused
                log.info("PAUSED - standing still; r recenters, s resumes." if paused else "FOLLOWING your head.")
                if paused:
                    self._hold()
            s = receiver.latest()
            if s is None:
                if tick - last_log > 2.0:
                    log.warning("no head-tracker samples (is the bridge running on port %s?)", receiver._addr[1])
                    last_log = tick
                if gizmo:
                    gizmo.pump(None, None, receiver.packets, self._status(paused))
                if self._sim_win is not None:
                    event = self._sim_win.pump(self._status(paused))
                    if event in ("estop", "estop_now"):
                        self.estop(immediate=event == "estop_now")
                        return
                    if event == "quit":
                        return
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

            if touchpad is not None:
                orientation = EulerZXY(yaw=target.yaw_rad, roll=target.roll_rad, pitch=target.pitch_rad)
                for action in touchpad.poll(tick, moving=tp_actions.stepping):
                    if paused and action.name not in (ESTOP, GRIPPER_TOGGLE):
                        log.info("touchpad: %s ignored while paused (press s to start)", action.name)
                        continue
                    tp_actions.run(action, orientation if not paused else None)
                    if action.name == ESTOP:
                        return

            # A step is a trajectory command; the per-tick stand command would
            # cancel it, so hold off until the step finishes or is aborted.
            if not paused and not (tp_actions is not None and tp_actions.stepping):
                self._send(target)

            for source, event in (("camera window", self._pump_viewer(target)),
                                  ("viz window", gizmo.pump(s, target, receiver.packets, self._status(paused)) if gizmo else None),
                                  ("sim window", self._sim_win.pump(self._status(paused)) if self._sim_win else None)):
                if event == "recenter":
                    need_recenter = True
                elif event == "walk":
                    self._toggle_walk()
                elif event == "toggle":
                    paused = not paused
                    log.info("PAUSED - standing still; r recenters, s resumes." if paused else "FOLLOWING your head.")
                    if paused:
                        self._hold()
                elif event == "estop":
                    self.estop(immediate=False)
                    return
                elif event == "estop_now":
                    self.estop(immediate=True)
                    return
                elif event == "quit":
                    log.info("quit from %s", source)
                    return

            if self.opts.dry_run or tick - last_log > 0.5:
                last_log = tick
                log.info(
                    "%s head y=%6.1f p=%6.1f r=%6.1f  ->  body y=%6.1f p=%6.1f r=%6.1f  v_rot=%5.2f",
                    "[paused]" if paused else "        ",
                    s.yaw, s.pitch, s.roll, target.yaw_deg, target.pitch_deg, target.roll_deg, target.v_rot_rad_s,
                )
            time.sleep(max(0.0, period - (time.monotonic() - tick)))

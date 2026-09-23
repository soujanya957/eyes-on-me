"""A kinematic MuJoCo stand-in for Spot, driven by the real SDK commands.

:class:`SimSpot` answers the two calls the control loop makes on the robot -
``robot_command(cmd)`` and ``get_robot_state()`` - with the same protobufs a
real Spot takes and returns. So everything upstream (head mapping, the
``w`` stand/walk switch, touchpad steps, e-stops) runs unchanged, and what
you see in the sim is what those exact commands would ask of the robot.

What it is **not** is a physics simulation of Spot. Spot's walking comes
from Boston Dynamics' controller, which there is no model of here. Instead
each tick poses the model directly:

* the **footprint** (x, y, yaw on the ground) integrates the commanded
  turn / step with speed and acceleration limits,
* the **body** sits above it at stand height, rotated by the commanded
  ``footprint_R_body`` offset (slewed, like the real body),
* the **feet** stay planted and the legs are solved by IK to reach them;
  when the footprint moves, diagonal pairs swing to new footholds (a trot),
* the **arm** is solved by IK to the commanded hand pose.

Good for trying the head-to-robot mapping and the controls. It says nothing
about balance, slipping, or what the real controller does near its limits.
"""

from __future__ import annotations

import itertools
import math
import time
from dataclasses import dataclass

import mujoco
import numpy as np
from bosdyn.api import arm_command_pb2, geometry_pb2, robot_state_pb2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.math_helpers import Quat, SE3Pose

from .model import load_model

LEGS = ("fl", "fr", "hl", "hr")
FEET = ("FL", "FR", "HL", "HR")
ARM = ("arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1")
GRIPPER = "arm_f1x"
#: Diagonal pairs swing together: a trot.
GAIT_PAIRS = ((0, 3), (1, 2))

#: Where the hand goes for the SDK's "ready" pose, in the flat body frame.
READY_HAND = (0.55, 0.0, 0.30)


@dataclass
class SimLimits:
    lin_speed: float = 0.6        # m/s, footprint
    yaw_rate: float = 1.2         # rad/s, footprint
    lin_accel: float = 1.5        # m/s^2
    yaw_accel: float = 3.0        # rad/s^2
    body_rate: float = 1.5        # rad/s, body orientation slew
    height_rate: float = 0.35     # m/s, stand / sit
    swing_s: float = 0.28         # one trot half-cycle
    step_height: float = 0.08     # m, foot lift
    replant_m: float = 0.03       # a foot this far off its nominal spot is re-placed
    arm_joint_speed: float = 2.5  # rad/s
    gripper_speed: float = 4.0    # rad/s
    traj_gain: float = 2.0        # 1/s, go-to-goal proportional gain
    traj_tol_m: float = 0.03
    traj_tol_rad: float = 0.03


def _yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def _qmat(q: np.ndarray) -> np.ndarray:
    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, q)
    return m.reshape(3, 3)


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def _quat_of(rot: geometry_pb2.Quaternion) -> np.ndarray:
    q = np.array([rot.w, rot.x, rot.y, rot.z], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0, 0, 0])


class SimSpot:
    """SDK-shaped kinematic Spot. Call :meth:`step` once per control tick."""

    def __init__(self, limits: SimLimits | None = None, model: mujoco.MjModel | None = None):
        self.lim = limits or SimLimits()
        self.model = model or load_model()
        self.data = mujoco.MjData(self.model)
        self._ik = mujoco.MjData(self.model)  # scratch copy for the arm solve
        m = self.model

        self.body_id = m.body("body").id
        self.hand_id = m.body("arm_link_wr1").id
        self._foot_geoms = [m.geom(f).id for f in FEET]
        self._leg_q = [[m.joint(f"{leg}_{j}").qposadr[0] for j in ("hx", "hy", "kn")] for leg in LEGS]
        self._leg_v = [[m.joint(f"{leg}_{j}").dofadr[0] for j in ("hx", "hy", "kn")] for leg in LEGS]
        self._arm_q = [m.joint(j).qposadr[0] for j in ARM]
        self._arm_v = [m.joint(j).dofadr[0] for j in ARM]
        self._arm_range = np.array([m.jnt_range[m.joint(j).id] for j in ARM])
        self._grip_q = m.joint(GRIPPER).qposadr[0]
        self._leg_range = [np.array([m.jnt_range[m.joint(f"{leg}_{j}").id] for j in ("hx", "hy", "kn")])
                           for leg in LEGS]

        # The "home" keyframe is Menagerie's standing pose: stand height and
        # footholds (relative to the footprint) are read off it.
        mujoco.mj_resetDataKeyframe(m, self.data, m.key("home").id)
        mujoco.mj_kinematics(m, self.data)
        self.stand_height = float(self.data.qpos[2])
        self.sit_height = 0.20
        self._nominal = np.array([self.data.geom_xpos[g].copy() for g in self._foot_geoms])
        self._q_stow = self.data.qpos[self._arm_q].copy()

        # Footprint pose and velocity (world frame), body offset, feet.
        self.x = self.y = self.yaw = 0.0
        self._v = np.zeros(3)                   # vx, vy (footprint frame), wz
        self._body_q = np.array([1.0, 0, 0, 0])  # footprint_R_body, current
        self._body_q_goal = self._body_q.copy()
        self._body_dz = 0.0                      # commanded stand height offset
        self.height = self.sit_height
        self._feet = self._nominal_world()
        self._swing: tuple[int, float, np.ndarray] | None = None  # (pair, t0, start positions)
        self._next_pair = 0

        self.powered = False
        self.estopped = False
        self._mobility: tuple = ("sit",)
        self._arm_goal: tuple = ("joints", self._q_stow.copy())
        self._grip_goal = 0.0
        self._ids = itertools.count(1)
        self._t = None
        self._pose_model()

    # ------------------------------------------------------------ robot API
    def has_arm(self) -> bool:
        return True

    def power_on(self) -> None:
        """Motors on and stand up (the sim's ``power_on`` + ``blocking_stand``)."""
        self.powered = True
        self._mobility = ("stand",)

    def power_off(self) -> None:
        """Sit down and cut power once seated."""
        self._mobility = ("sit",)
        self._arm_goal = ("joints", self._q_stow.copy())

    def estop(self, immediate: bool) -> None:
        """Software e-stop. Cut-now slumps quickly; settle-then-cut sits first."""
        self.estopped = True
        self._mobility = ("slump",) if immediate else ("sit",)
        self._v[:] = 0.0

    def robot_command(self, command, end_time_secs: float | None = None, **_) -> int:
        if self.estopped:
            return next(self._ids)
        sync = command.synchronized_command
        if sync.HasField("mobility_command"):
            self._mobility_command(sync.mobility_command, end_time_secs)
        if sync.HasField("arm_command"):
            self._arm_command(sync.arm_command)
        if sync.HasField("gripper_command"):
            pts = sync.gripper_command.claw_gripper_command.trajectory.points
            if pts:
                self._grip_goal = float(np.clip(pts[-1].point, -1.5708, 0.0))
        return next(self._ids)

    def get_robot_state(self) -> robot_state_pb2.RobotState:
        """Just the transforms snapshot: odom (root), body, flat_body, vision."""
        snap = geometry_pb2.FrameTreeSnapshot()

        def edge(child: str, parent: str, pose: SE3Pose) -> None:
            e = snap.child_to_parent_edge_map[child]
            e.parent_frame_name = parent
            e.parent_tform_child.CopyFrom(pose.to_proto())

        pos, q = self.data.qpos[0:3], self.data.qpos[3:7]
        body_yaw = self._body_world_yaw()
        edge("odom", "", SE3Pose(0, 0, 0, Quat()))
        edge("vision", "odom", SE3Pose(0, 0, 0, Quat()))
        edge("body", "odom", SE3Pose(*pos, Quat(*q)))
        edge("flat_body", "odom", SE3Pose(*pos, Quat(*_yaw_quat(body_yaw))))
        return robot_state_pb2.RobotState(kinematic_state=robot_state_pb2.KinematicState(transforms_snapshot=snap))

    # ------------------------------------------------------- command parse
    def _mobility_command(self, mob, end_time_secs: float | None) -> None:
        if mob.HasField("params"):
            params = spot_command_pb2.MobilityParams()
            if mob.params.Unpack(params):
                pts = params.body_control.base_offset_rt_footprint.points
                if pts:
                    self._body_q_goal = _quat_of(pts[-1].pose.rotation)
                    self._body_dz = pts[-1].pose.position.z
        if not self.powered:
            return
        # SDK end times are wall-clock; the sim runs on its own step clock.
        now = self._t if self._t is not None else time.monotonic()
        until = math.inf if end_time_secs is None else now + (end_time_secs - time.time())
        if mob.HasField("stand_request"):
            self._mobility = ("stand",)
        elif mob.HasField("sit_request"):
            self._mobility = ("sit",)
        elif mob.HasField("se2_velocity_request"):
            v = mob.se2_velocity_request.velocity
            self._mobility = ("vel", v.linear.x, v.linear.y, v.angular, until)
        elif mob.HasField("se2_trajectory_request"):
            req = mob.se2_trajectory_request
            goal = req.trajectory.points[-1].pose
            self._mobility = ("traj", goal.position.x, goal.position.y, goal.angle, until)

    def _arm_command(self, arm) -> None:
        if not self.powered:
            return
        if arm.HasField("named_arm_position_command"):
            if arm.named_arm_position_command.position == arm_command_pb2.NamedArmPositionsCommand.POSITIONS_STOW:
                self._arm_goal = ("joints", self._q_stow.copy())
            else:  # READY, CARRY: hand out in front
                self._arm_goal = ("pose", np.array(READY_HAND), np.array([1.0, 0, 0, 0]))
        elif arm.HasField("arm_cartesian_command"):
            cart = arm.arm_cartesian_command
            pts = cart.pose_trajectory_in_task.points
            if pts:
                p = pts[-1].pose
                self._arm_goal = ("pose", np.array([p.position.x, p.position.y, p.position.z]),
                                  _quat_of(p.rotation))

    # ------------------------------------------------------------ stepping
    def step(self, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        dt = 0.0 if self._t is None else min(now - self._t, 0.1)
        self._t = now
        if dt > 0:
            self._step_mobility(now, dt)
            self._step_body(dt)
            self._step_feet(now)
            self._step_arm(dt)
        self._pose_model()

    def _desired_velocity(self, now: float) -> np.ndarray:
        kind = self._mobility[0]
        if kind == "vel":
            _, vx, vy, wz, until = self._mobility
            if now > until:
                self._mobility = ("stand",)
                return np.zeros(3)
            return np.array([vx, vy, wz])
        if kind == "traj":
            _, gx, gy, gyaw, until = self._mobility
            dx, dy, dyaw = gx - self.x, gy - self.y, _wrap(gyaw - self.yaw)
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            ex, ey = c * dx + s * dy, -s * dx + c * dy
            if (math.hypot(ex, ey) < self.lim.traj_tol_m and abs(dyaw) < self.lim.traj_tol_rad) \
                    or now > until:
                self._mobility = ("stand",)
                return np.zeros(3)
            k = self.lim.traj_gain
            return np.array([k * ex, k * ey, k * dyaw])
        return np.zeros(3)

    def _step_mobility(self, now: float, dt: float) -> None:
        want = self._desired_velocity(now)
        lin = np.hypot(want[0], want[1])
        if lin > self.lim.lin_speed:
            want[:2] *= self.lim.lin_speed / lin
        want[2] = float(np.clip(want[2], -self.lim.yaw_rate, self.lim.yaw_rate))
        dv = want - self._v
        dv[:2] = np.clip(dv[:2], -self.lim.lin_accel * dt, self.lim.lin_accel * dt)
        dv[2] = float(np.clip(dv[2], -self.lim.yaw_accel * dt, self.lim.yaw_accel * dt))
        self._v += dv
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (c * self._v[0] - s * self._v[1]) * dt
        self.y += (s * self._v[0] + c * self._v[1]) * dt
        self.yaw = _wrap(self.yaw + self._v[2] * dt)

        kind = self._mobility[0]
        if kind in ("sit", "slump"):
            goal, rate = self.sit_height, self.lim.height_rate * (3.0 if kind == "slump" else 1.0)
        else:
            goal, rate = self.stand_height + self._body_dz, self.lim.height_rate
        self.height += float(np.clip(goal - self.height, -rate * dt, rate * dt))
        if kind in ("sit", "slump") and abs(self.height - self.sit_height) < 1e-3:
            self.powered = False

    def _step_body(self, dt: float) -> None:
        goal = self._body_q_goal if self._mobility[0] not in ("sit", "slump") else np.array([1.0, 0, 0, 0])
        dv = np.zeros(3)
        mujoco.mju_subQuat(dv, goal, self._body_q)
        n = np.linalg.norm(dv)
        cap = self.lim.body_rate * dt
        if n > cap:
            dv *= cap / n
        mujoco.mju_quatIntegrate(self._body_q, dv, 1.0)
        mujoco.mju_normalize4(self._body_q)

    def _nominal_world(self, x=None, y=None, yaw=None) -> np.ndarray:
        x = self.x if x is None else x
        y = self.y if y is None else y
        yaw = self.yaw if yaw is None else yaw
        c, s = math.cos(yaw), math.sin(yaw)
        out = self._nominal.copy()
        out[:, 0] = x + c * self._nominal[:, 0] - s * self._nominal[:, 1]
        out[:, 1] = y + s * self._nominal[:, 0] + c * self._nominal[:, 1]
        return out

    def _step_feet(self, now: float) -> None:
        if self._mobility[0] in ("sit", "slump") and self._swing is None:
            return
        moving = np.linalg.norm(self._v[:2]) > 0.02 or abs(self._v[2]) > 0.05
        # Aim each swing where the foot belongs half a swing from now, so the
        # gait keeps up with the body instead of trailing it.
        ahead = self.lim.swing_s * 0.5
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        target = self._nominal_world(self.x + (c * self._v[0] - s * self._v[1]) * ahead,
                                     self.y + (s * self._v[0] + c * self._v[1]) * ahead,
                                     self.yaw + self._v[2] * ahead)
        if self._swing is not None:
            pair, t0, start = self._swing
            u = min(1.0, (now - t0) / self.lim.swing_s)
            e = u * u * (3 - 2 * u)  # smoothstep
            for k, i in enumerate(GAIT_PAIRS[pair]):
                p = start[k] + (target[i] - start[k]) * e
                p[2] = self._nominal[i, 2] + self.lim.step_height * math.sin(math.pi * u)
                self._feet[i] = p
            if u >= 1.0:
                self._swing = None
            return
        err = np.linalg.norm((target - self._feet)[:, :2], axis=1)
        if not self.powered or not (moving or err.max() > self.lim.replant_m):
            return
        if moving:
            pair, self._next_pair = self._next_pair, 1 - self._next_pair
        else:
            pair = 0 if max(err[i] for i in GAIT_PAIRS[0]) >= max(err[i] for i in GAIT_PAIRS[1]) else 1
        self._swing = (pair, now, np.array([self._feet[i].copy() for i in GAIT_PAIRS[pair]]))

    # ----------------------------------------------------------------- arm
    def _flat_body(self) -> tuple[np.ndarray, np.ndarray]:
        """flat_body in world: body position, heading only."""
        return self.data.qpos[0:3].copy(), _yaw_quat(self._body_world_yaw())

    def _body_world_yaw(self) -> float:
        r = _qmat(self.data.qpos[3:7])
        return math.atan2(r[1, 0], r[0, 0])

    def _step_arm(self, dt: float) -> None:
        q = self.data.qpos[self._arm_q].copy()
        if self._arm_goal[0] == "joints" or not self.powered:
            goal = self._arm_goal[1] if self._arm_goal[0] == "joints" else q
        else:
            _, p_fb, q_fb = self._arm_goal
            fb_p, fb_q = self._flat_body()
            goal = self._solve_arm(fb_p + _qmat(fb_q) @ p_fb, _qmul(fb_q, q_fb))
        cap = self.lim.arm_joint_speed * dt
        self.data.qpos[self._arm_q] = q + np.clip(goal - q, -cap, cap)
        g = self.data.qpos[self._grip_q]
        cap = self.lim.gripper_speed * dt
        self.data.qpos[self._grip_q] = g + float(np.clip(self._grip_goal - g, -cap, cap))

    def _solve_arm(self, p_goal: np.ndarray, q_goal: np.ndarray, iters: int = 25) -> np.ndarray:
        """Damped least squares on a scratch copy; warm-started, falls back to a ready seed."""
        m, d = self.model, self._ik
        d.qpos[:] = self.data.qpos
        q = self.data.qpos[self._arm_q]
        stowed = np.linalg.norm(q - self._q_stow) < 0.3
        if stowed:  # folded arm is near-singular: start from an unfolded seed
            d.qpos[self._arm_q] = [0.0, -1.0, 1.8, 0.0, -0.8, 0.0]
        jacp, jacr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        err = np.zeros(6)
        dq_rot = np.zeros(3)
        for _ in range(iters):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            err[:3] = p_goal - d.xpos[self.hand_id]
            mujoco.mju_subQuat(dq_rot, q_goal, d.xquat[self.hand_id])
            err[3:] = 0.5 * (d.xmat[self.hand_id].reshape(3, 3) @ dq_rot)
            if np.linalg.norm(err) < 1e-4:
                break
            mujoco.mj_jacBody(m, d, jacp, jacr, self.hand_id)
            j = np.vstack([jacp[:, self._arm_v], 0.5 * jacr[:, self._arm_v]])
            dq = j.T @ np.linalg.solve(j @ j.T + 0.05 ** 2 * np.eye(6), err)
            d.qpos[self._arm_q] = np.clip(d.qpos[self._arm_q] + dq, self._arm_range[:, 0], self._arm_range[:, 1])
        return d.qpos[self._arm_q].copy()

    # ---------------------------------------------------------------- pose
    def _pose_model(self) -> None:
        """Body from footprint + offset, then legs by IK to the planted feet."""
        m, d = self.model, self.data
        d.qpos[0:3] = (self.x, self.y, self.height)
        d.qpos[3:7] = _qmul(_yaw_quat(self.yaw), self._body_q)
        jacp = np.zeros((3, m.nv))
        for _ in range(6):
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            worst = 0.0
            for leg in range(4):
                g = self._foot_geoms[leg]
                err = self._feet[leg] - d.geom_xpos[g]
                worst = max(worst, float(np.abs(err).max()))
                mujoco.mj_jacGeom(m, d, jacp, None, g)
                j = jacp[:, self._leg_v[leg]]
                dq = j.T @ np.linalg.solve(j @ j.T + 0.02 ** 2 * np.eye(3), err)
                adr = self._leg_q[leg]
                d.qpos[adr] = np.clip(d.qpos[adr] + dq, self._leg_range[leg][:, 0], self._leg_range[leg][:, 1])
            if worst < 5e-4:
                break
        mujoco.mj_kinematics(m, d)

    # ------------------------------------------------------------- queries
    @property
    def footprint(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def hand_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """Hand position and rotation matrix (x = pointing direction), world frame."""
        return self.data.xpos[self.hand_id].copy(), self.data.xmat[self.hand_id].reshape(3, 3).copy()

    def front_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """A camera spot at the body's nose, looking along the body's x axis."""
        r = self.data.xmat[self.body_id].reshape(3, 3).copy()
        return self.data.xpos[self.body_id] + r @ np.array([0.5, 0.0, 0.05]), r

    @property
    def arm_out(self) -> bool:
        return np.linalg.norm(self.data.qpos[self._arm_q] - self._q_stow) > 0.3

"""The MuJoCo stand-in answers real SDK commands the way the control loop expects.

Needs the Menagerie model, fetched on the first ``eyes-on-me --sim``; skipped
until then so the suite never touches the network.
"""

import math
import time

import numpy as np
import pytest

pytest.importorskip("mujoco")
from bosdyn.client.frame_helpers import get_odom_tform_body  # noqa: E402
from bosdyn.client.robot_command import RobotCommandBuilder as B  # noqa: E402
from bosdyn.geometry import EulerZXY  # noqa: E402

from eyes_on_me.sim import model as sim_model  # noqa: E402

if not (sim_model.DEFAULT_CACHE / "boston_dynamics_spot" / "scene_arm.xml").is_file():
    pytest.skip("Spot model not fetched yet (run `eyes-on-me --sim` once)", allow_module_level=True)

from eyes_on_me.sim import SimSpot  # noqa: E402


@pytest.fixture(scope="module")
def model():
    return sim_model.load_model()


def run(sim, t0, seconds, dt=0.05):
    t = t0
    for _ in range(int(seconds / dt)):
        t += dt
        sim.step(t)
    return t


def feet_error(sim):
    return max(float(np.linalg.norm(sim._feet[i] - sim.data.geom_xpos[g])) for i, g in enumerate(sim._foot_geoms))


def params(yaw=0.0, pitch=0.0, roll=0.0):
    return B.mobility_params(body_height=0.0, footprint_R_body=EulerZXY(yaw=yaw, roll=roll, pitch=pitch))


def test_stands_and_leans_with_feet_planted(model):
    sim = SimSpot(model=model)
    t = run(sim, 0.0, 0.2)
    sim.robot_command(B.synchro_stand_command(params=params(yaw=0.3)))
    assert sim.height == pytest.approx(sim.sit_height)  # unpowered: nothing moves
    sim.power_on()
    sim.robot_command(B.synchro_stand_command(params=params(yaw=0.3, pitch=0.2)))
    run(sim, t, 1.5)
    assert sim.height == pytest.approx(sim.stand_height, abs=1e-3)
    snap = sim.get_robot_state().kinematic_state.transforms_snapshot
    assert get_odom_tform_body(snap).rot.to_yaw() == pytest.approx(0.3, abs=0.01)
    assert (sim.x, sim.y, sim.yaw) == (0.0, 0.0, 0.0)  # leaning never moves the feet
    assert feet_error(sim) < 1e-3


def test_velocity_turns_then_expires(model):
    sim = SimSpot(model=model)
    sim.power_on()
    t = run(sim, 0.0, 1.0)
    for _ in range(40):
        sim.robot_command(B.synchro_velocity_command(v_x=0, v_y=0, v_rot=0.8, params=params()),
                          end_time_secs=time.time() + 0.2)
        t = run(sim, t, 0.05)
    turned = sim.yaw
    assert math.degrees(turned) > 60
    t = run(sim, t, 1.5)  # no more commands: the last one expires and Spot stops
    assert sim._mobility == ("stand",)
    assert math.degrees(sim.yaw - turned) < 15
    assert feet_error(sim) < 1e-3


def test_touchpad_step_goes_forward_along_heading(model):
    sim = SimSpot(model=model)
    sim.power_on()
    sim.yaw = math.pi / 2  # facing +y
    t = run(sim, 0.0, 1.0)
    snap = sim.get_robot_state().kinematic_state.transforms_snapshot
    sim.robot_command(B.synchro_trajectory_command_in_body_frame(0.5, 0.0, 0.0, snap, params=params()),
                      end_time_secs=time.time() + 5)
    run(sim, t, 3.0)
    assert sim.y == pytest.approx(0.5, abs=0.05)
    assert sim.x == pytest.approx(0.0, abs=0.05)


def test_arm_reaches_hand_pose_and_gripper_opens(model):
    from eyes_on_me.sim.spot_sim import _qmat

    sim = SimSpot(model=model)
    sim.power_on()
    t = run(sim, 0.0, 1.0)
    sim.robot_command(B.arm_pose_command(0.6, 0.0, 0.55, 1, 0, 0, 0, "flat_body", seconds=1.0))
    sim.robot_command(B.claw_gripper_open_command())
    run(sim, t, 3.0)
    p, _ = sim.hand_pose()
    fb_p, fb_q = sim._flat_body()
    assert _qmat(fb_q).T @ (p - fb_p) == pytest.approx([0.6, 0.0, 0.55], abs=0.01)
    assert sim.data.qpos[sim._grip_q] == pytest.approx(-1.5708, abs=1e-3)


def test_estop_sits_cuts_and_ignores_commands(model):
    sim = SimSpot(model=model)
    sim.power_on()
    t = run(sim, 0.0, 1.5)
    sim.estop(immediate=False)
    sim.robot_command(B.synchro_velocity_command(v_x=0, v_y=0, v_rot=1.0), end_time_secs=time.time() + 5)
    run(sim, t, 2.0)
    assert not sim.powered
    assert sim.height == pytest.approx(sim.sit_height, abs=1e-3)
    assert sim.yaw == 0.0

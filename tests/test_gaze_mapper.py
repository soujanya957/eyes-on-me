import math
import os

import pytest

from eyes_on_me.gaze_mapper import GazeConfig, GazeMapper, apply_deadband, wrap_deg
from eyes_on_me.head_tracker import parse_sample


def mapper(**kw):
    """Mapper for the mechanics tests below (clamping, recentre, turn split).

    Yaw inversion is incidental to those, and un-inverted yaw keeps the
    expectations readable, so it is off here. The shipped defaults are pinned
    separately by :func:`test_default_axis_convention`.
    """
    kw.setdefault("invert_yaw", False)
    return GazeMapper(GazeConfig(smoothing=1.0, deadband_deg=0.0, **kw))


def test_default_axis_convention():
    """The signs eyes-on-me ships with, measured on the WH-1000XM5 against Spot.

    Look right -> body yaws right (Spot +yaw is nose *left*, so negative);
    look up -> nose up (Spot +pitch is nose *down*, so negative); roll is not
    inverted and carries the 0.5 roll gain.
    """
    m = GazeMapper(GazeConfig(smoothing=1.0, deadband_deg=0.0))
    t = m.pose_target(yaw=10, pitch=10, roll=10)
    assert t.yaw_deg == -10.0
    assert t.pitch_deg == -10.0
    assert t.roll_deg == 5.0


def test_wrap():
    assert wrap_deg(190) == -170
    assert wrap_deg(-190) == 170
    assert wrap_deg(180) == -180


def test_deadband_is_continuous():
    assert apply_deadband(1.0, 2.0) == 0.0
    assert apply_deadband(2.0, 2.0) == 0.0
    assert math.isclose(apply_deadband(5.0, 2.0), 3.0)
    assert math.isclose(apply_deadband(-5.0, 2.0), -3.0)


def test_pose_clamps_and_inverts_pitch():
    m = mapper()
    t = m.pose_target(yaw=60, pitch=10, roll=0)
    assert t.yaw_deg == 25.0  # clamped to max_body_yaw
    assert t.pitch_deg == -10.0  # tracker up -> Spot nose-up (negative pitch)
    assert t.v_rot_rad_s == 0.0


def test_recenter_makes_current_pose_zero():
    m = mapper()
    m.recenter(yaw=30, pitch=-5, roll=2)
    t = m.pose_target(30, -5, 2)
    assert (t.yaw_deg, t.pitch_deg, t.roll_deg) == (0.0, 0.0, 0.0)
    t = m.pose_target(40, -5, 2)
    assert t.yaw_deg == 10.0


def test_turn_splits_body_and_rotation():
    m = mapper()
    # Head 60 deg left, robot hasn't turned yet: body takes 25, remainder drives rotation.
    t = m.turn_target(60, 0, 0, robot_heading_rel_deg=0.0)
    assert t.yaw_deg == 25.0
    assert t.v_rot_rad_s > 0
    # Robot has turned 40 deg: remaining 20 fits in body pose, no rotation.
    t = m.turn_target(60, 0, 0, robot_heading_rel_deg=40.0)
    assert math.isclose(t.yaw_deg, 20.0)
    assert t.v_rot_rad_s == 0.0


def test_turn_rate_is_capped():
    m = mapper()
    t = m.turn_target(179, 0, 0, 0.0)
    assert t.v_rot_rad_s == GazeConfig().max_turn_rate_rad_s


def test_smoothing_converges():
    m = GazeMapper(GazeConfig(smoothing=0.5, deadband_deg=0.0, invert_yaw=False))
    for _ in range(20):
        t = m.pose_target(10, 0, 0)
    assert math.isclose(t.yaw_deg, 10.0, abs_tol=1e-3)


def test_parse_sample_protocol_example():
    raw = (b'{"version":2,"device":"WH-1000XM5","rotationVector":[0.012,-0.004,0.31],'
           b'"quaternion":[0.987,0.006,-0.002,0.155],"yprDegrees":[17.84,-0.46,1.37],'
           b'"gyroscope":[0.01,0.0,-0.02],"accelerometer":null,"resetCounter":0,'
           b'"packetsPerSecond":25.0,"receiveLatencyMs":-1.0}')
    s = parse_sample(raw, now=0.0)
    assert s is not None
    assert (s.yaw, s.pitch, s.roll) == (17.84, -0.46, 1.37)
    assert parse_sample(b"not json") is None
    assert parse_sample(b'{"version":2}') is None


def test_arm_uses_wider_envelope():
    m = mapper()
    t = m.arm_target(yaw=60, pitch=40, roll=0)
    assert t.yaw_deg == 60.0  # fits in ±70 arm yaw, would clamp to 25 for the body
    assert t.pitch_deg == -40.0
    assert m.arm_target(yaw=100, pitch=0, roll=0).yaw_deg == 70.0


def test_camera_for_yaw():
    from eyes_on_me.camera import camera_for_yaw
    assert camera_for_yaw(0) == "front"
    assert camera_for_yaw(-5) == "front-right"
    assert camera_for_yaw(60) == "left"
    assert camera_for_yaw(-60) == "right"
    assert camera_for_yaw(170) == "back"


def test_shift_yaw_rezeroes_on_the_walked_heading():
    """Standing again after walking: the heading Spot turned to becomes straight ahead."""
    for m in (mapper(), GazeMapper(GazeConfig())):  # plain, and shipped (inverted yaw)
        m.recenter(10, 0, 0)
        walked = m.head_to_body(40, 0, 0)[0]  # pretend Spot turned all the way to face the head
        m.shift_yaw(walked)
        assert abs(m.head_to_body(40, 0, 0)[0]) < 1e-9


def test_robot_selection(tmp_path, monkeypatch):
    from eyes_on_me import cli
    argv = ["run", "--robot", "rooter", "--viz"]
    assert cli.pop_robot(argv) == "rooter" and argv == ["run", "--viz"]
    argv = ["--robot=tusker"]
    assert cli.pop_robot(argv) == "tusker" and argv == []

    (tmp_path / ".env").write_text("SPOT_ROBOT=tusker\n")
    (tmp_path / ".env.tusker").write_text("SPOT_IP=10.0.0.1\nSPOT_USER=t\n")
    (tmp_path / ".env.rooter").write_text("SPOT_IP=10.0.0.2\nSPOT_USER=r\n")
    monkeypatch.setattr(cli, "REPO", tmp_path)
    monkeypatch.chdir(tmp_path)
    for k in ("SPOT_ROBOT", "SPOT_IP", "SPOT_USER", "BOSDYN_CLIENT_USERNAME"):
        monkeypatch.delenv(k, raising=False)
    assert cli.load_env() == "tusker"
    assert os.environ["SPOT_IP"] == "10.0.0.1"
    for k in ("SPOT_ROBOT", "SPOT_IP", "SPOT_USER", "BOSDYN_CLIENT_USERNAME"):
        monkeypatch.delenv(k, raising=False)
    assert cli.load_env("rooter") == "rooter"
    assert os.environ["SPOT_IP"] == "10.0.0.2" and os.environ["BOSDYN_CLIENT_USERNAME"] == "r"
    assert cli.robots() == ["rooter", "tusker"]
    with pytest.raises(SystemExit, match="known robots: rooter, tusker"):
        cli.load_env("nope")

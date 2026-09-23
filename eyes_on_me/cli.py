"""``eyes-on-me`` command line entry point."""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

from dotenv import dotenv_values, load_dotenv

from .camera import SOURCES
from .gaze_mapper import GazeConfig, GazeMapper
from .head_tracker import HeadTrackerReceiver
from .spot_gaze import RunOptions, SpotGaze


SUBCOMMANDS = ("run", "viz", "check", "touchpad-test")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="eyes-on-me run",
        description="Point Spot's body where your head is pointing, using Sony headphone head tracking. "
                    "Other subcommands: `eyes-on-me viz` (headphones only), `eyes-on-me check` (robot pre-flight).",
    )
    p.add_argument("hostname", nargs="?", default=os.environ.get("SPOT_IP"),
                   help="Spot IP/hostname (default: $SPOT_IP of the chosen robot; omit with --dry-run)")
    p.add_argument("--robot", metavar="NAME",
                   help="which Spot: loads .env.NAME (e.g. tusker, rooter). Default: $SPOT_ROBOT from .env")
    p.add_argument("--mode", choices=["pose", "turn", "arm"], default="pose",
                   help="pose: body tilt only (default, robot stays put). turn: also rotate in place for large yaw. "
                        "arm: raise the gripper and point it where you look (needs Spot Arm).")
    p.add_argument("--camera", choices=["none", "auto", *SOURCES], default=None,
                   help="show a Spot camera in a window. auto follows head yaw. Default: hand in arm mode, else none. "
                        "Window keys: 1-6 pick camera, 0 auto, r recenter, q quit.")
    p.add_argument("--dry-run", action="store_true", help="print mapped body targets; never talk to Spot")
    p.add_argument("--sim", action="store_true",
                   help="drive a MuJoCo Spot in a window instead of a robot (headphones still needed, or "
                        "scripts/fake-tracker.py). Kinematic: shows the commands, not real walking physics.")
    p.add_argument("--viz", action="store_true",
                   help="show the head gizmo next to the robot (one process: `eyes-on-me viz` cannot run alongside, "
                        "both would bind the same UDP port)")
    p.add_argument("--touchpad", action="store_true",
                   help="WH-1000XM5 earcup gestures: swipe fwd/back = step fwd/back, swipe up/down = step "
                        "left/right, double tap = gripper (or abort a step), double tap x5 = E-STOP. "
                        "Disconnect your phone first - with multipoint the headset sends gestures there.")
    p.add_argument("--start-active", action="store_true",
                   help="follow the head straight away; default is to stand still until you press s")
    p.add_argument("--udp-port", type=int, default=4243, help="sony-head-tracker JSON port (bridge --port + 1)")
    p.add_argument("--rate", type=float, default=20.0, help="command rate, Hz")
    p.add_argument("--body-height", type=float, default=0.0, help="stand height offset, m")
    p.add_argument("--external-estop", action="store_true",
                   help="do not register an e-stop endpoint; rely on the tablet or another client")
    p.add_argument("--no-sit", action="store_true", help="leave Spot standing on exit")
    p.add_argument("--no-recenter", action="store_true", help="don't treat the first sample as straight-ahead")
    p.add_argument("--hand-pos", type=float, nargs=3, metavar=("X", "Y", "Z"), default=(0.6, 0.0, 0.55),
                   help="arm mode: where the gripper hovers, metres in the flat body frame (default 0.6 0 0.55)")

    g = p.add_argument_group("mapping")
    d = GazeConfig()
    g.add_argument("--max-yaw", type=float, default=d.max_body_yaw_deg, help="deg")
    g.add_argument("--max-pitch", type=float, default=d.max_body_pitch_deg, help="deg")
    g.add_argument("--max-roll", type=float, default=d.max_body_roll_deg, help="deg")
    g.add_argument("--arm-max-yaw", type=float, default=d.max_arm_yaw_deg, help="deg, arm mode")
    g.add_argument("--arm-max-pitch", type=float, default=d.max_arm_pitch_deg, help="deg, arm mode")
    g.add_argument("--arm-max-roll", type=float, default=d.max_arm_roll_deg, help="deg, arm mode")
    g.add_argument("--yaw-gain", type=float, default=d.yaw_gain)
    g.add_argument("--pitch-gain", type=float, default=d.pitch_gain)
    g.add_argument("--roll-gain", type=float, default=d.roll_gain)
    g.add_argument("--deadband", type=float, default=d.deadband_deg, help="deg")
    g.add_argument("--smoothing", type=float, default=d.smoothing, help="EMA alpha, 1.0 = off")
    g.add_argument("--no-invert-yaw", action="store_true", help="tracker yaw is inverted by default")
    g.add_argument("--no-invert-pitch", action="store_true", help="tracker pitch is inverted by default")
    g.add_argument("--invert-roll", action="store_true", default=d.invert_roll)
    g.add_argument("--turn-kp", type=float, default=d.turn_kp)
    g.add_argument("--max-turn-rate", type=float, default=d.max_turn_rate_rad_s, help="rad/s")
    return p


REPO = Path(__file__).resolve().parent.parent


def pop_robot(argv: list[str]) -> str | None:
    """Remove ``--robot NAME`` / ``--robot=NAME`` from ``argv`` and return NAME.

    Read before any parser is built, because the parsers take their hostname
    default from ``$SPOT_IP``, which depends on which robot's file is loaded.
    """
    for i, a in enumerate(argv):
        if a == "--robot" and i + 1 < len(argv):
            name = argv[i + 1]
            del argv[i:i + 2]
            return name
        if a.startswith("--robot="):
            del argv[i]
            return a.split("=", 1)[1]
    return None


def robots() -> list[str]:
    return sorted(p.name.split(".", 2)[2] for p in REPO.glob(".env.*") if p.name != ".env.example")


def load_env(robot: str | None = None) -> str | None:
    """Load the chosen robot's ``.env.<name>``, then map SPOT_* onto the SDK's variables.

    The robot is ``--robot``, else ``$SPOT_ROBOT`` (usually set in the plain
    ``.env``). Returns the robot name, or None when running from a plain
    ``.env`` that holds SPOT_IP itself.

    ``bosdyn.client.util.authenticate`` reads BOSDYN_CLIENT_USERNAME/PASSWORD;
    SPOT_USER/SPOT_PASS are friendlier names for the same thing. Existing
    environment variables always win over the files.
    """
    real_env = set(os.environ)
    load_dotenv(REPO / ".env")
    load_dotenv()  # also honour a .env in the current directory
    robot = robot or os.environ.get("SPOT_ROBOT")
    if robot:
        path = REPO / f".env.{robot}"
        if not path.is_file():
            known = ", ".join(robots()) or "none"
            raise SystemExit(f"error: no {path.name} for robot {robot!r} (known robots: {known})")
        # The robot file wins over the plain .env, but not over the real environment.
        for k, v in dotenv_values(path).items():
            if k not in real_env and v is not None:
                os.environ[k] = v
    for src, dst in (("SPOT_USER", "BOSDYN_CLIENT_USERNAME"), ("SPOT_PASS", "BOSDYN_CLIENT_PASSWORD")):
        if src in os.environ and dst not in os.environ:
            os.environ[dst] = os.environ[src]
    return robot


def cfg_from_args(args: argparse.Namespace) -> GazeConfig:
    return GazeConfig(
        max_body_yaw_deg=args.max_yaw,
        max_body_pitch_deg=args.max_pitch,
        max_body_roll_deg=args.max_roll,
        max_arm_yaw_deg=args.arm_max_yaw,
        max_arm_pitch_deg=args.arm_max_pitch,
        max_arm_roll_deg=args.arm_max_roll,
        yaw_gain=args.yaw_gain,
        pitch_gain=args.pitch_gain,
        roll_gain=args.roll_gain,
        deadband_deg=args.deadband,
        smoothing=args.smoothing,
        invert_yaw=not args.no_invert_yaw,
        invert_pitch=not args.no_invert_pitch,
        invert_roll=args.invert_roll,
        turn_kp=args.turn_kp,
        max_turn_rate_rad_s=args.max_turn_rate,
    )


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    robot = load_env(pop_robot(argv))
    sub = argv.pop(0) if argv and argv[0] in SUBCOMMANDS else "run"
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    if robot and sub == "check":
        logging.getLogger("eyes_on_me").info("robot: %s (%s)", robot, os.environ.get("SPOT_IP", "no SPOT_IP"))

    if sub == "viz":
        from .viz import main as viz_main

        vp = argparse.ArgumentParser(prog="eyes-on-me viz", description="Head-tracking gizmo; no robot needed.",
                                     parents=[build_parser()], add_help=False, conflict_handler="resolve")
        vp.add_argument("--arm", action="store_true", help="show the arm envelope instead of the body envelope")
        va = vp.parse_args(argv)
        return viz_main(va.udp_port, cfg_from_args(va), arm=va.arm)

    if sub == "touchpad-test":
        from .touchpad.gestures import GestureConfig
        from .touchpad.tester import main as touchpad_main

        tp = argparse.ArgumentParser(prog="eyes-on-me touchpad-test",
                                     description="Print the actions your touchpad gestures would send to Spot. Moves nothing.")
        tp.add_argument("--moving", action="store_true",
                        help="pretend a step is always running, so a double tap aborts instead of toggling the gripper")
        tp.add_argument("--tap-settle", type=float, default=GestureConfig().tap_settle_s,
                        help="seconds an idle double tap waits, in case more follow (e-stop sequence)")
        tp.add_argument("--debounce", type=float, default=GestureConfig().debounce_s, help="seconds")
        ta = tp.parse_args(argv)
        return touchpad_main(GestureConfig(debounce_s=ta.debounce, tap_settle_s=ta.tap_settle), ta.moving)

    if sub == "check":
        from .check import main as check_main

        cp = argparse.ArgumentParser(prog="eyes-on-me check", description="Spot pre-flight: network, auth, e-stop, lease, power, arm, cameras.")
        cp.add_argument("hostname", nargs="?", default=os.environ.get("SPOT_IP"))
        cp.add_argument("--robot", metavar="NAME", help="which Spot: loads .env.NAME")
        cp.add_argument("--udp-port", type=int, default=4243)
        ca = cp.parse_args(argv)
        if not ca.hostname:
            print("error: give a hostname or pick a robot with --robot", file=sys.stderr)
            return 2
        return check_main(ca.hostname, ca.udp_port)

    args = build_parser().parse_args(argv)
    if robot and not (args.sim or args.dry_run):
        logging.getLogger("eyes_on_me").info("robot: %s (%s)", robot, args.hostname or "no SPOT_IP")
    if not args.dry_run and not args.sim and not args.hostname:
        print("error: give a hostname or pick a robot with --robot (or use --dry-run)", file=sys.stderr)
        return 2

    cfg = cfg_from_args(args)
    camera = args.camera if args.camera is not None else ("hand" if args.mode == "arm" and not args.sim else "none")
    opts = RunOptions(
        mode=args.mode,
        camera=camera,
        rate_hz=args.rate,
        body_height=args.body_height,
        dry_run=args.dry_run,
        external_estop=args.external_estop,
        sit_on_exit=not args.no_sit,
        recenter_on_start=not args.no_recenter,
        viz=args.viz,
        start_paused=not args.start_active,
        touchpad=args.touchpad,
        sim=args.sim,
        hand_x=args.hand_pos[0],
        hand_y=args.hand_pos[1],
        hand_z=args.hand_pos[2],
    )

    receiver = HeadTrackerReceiver(port=args.udp_port).start()
    spot = SpotGaze(args.hostname or "", GazeMapper(cfg), opts)
    try:
        spot.connect()
        spot.run(receiver)
    except KeyboardInterrupt:
        pass
    finally:
        receiver.stop()
        spot.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""``eyes-on-me`` command line entry point."""

from __future__ import annotations

import argparse
import logging
import os
import sys
from pathlib import Path

from dotenv import load_dotenv

from .camera import SOURCES
from .gaze_mapper import GazeConfig, GazeMapper
from .head_tracker import HeadTrackerReceiver
from .spot_gaze import RunOptions, SpotGaze


SUBCOMMANDS = ("run", "viz", "check")


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="eyes-on-me run",
        description="Point Spot's body where your head is pointing, using Sony headphone head tracking. "
                    "Other subcommands: `eyes-on-me viz` (headphones only), `eyes-on-me check` (robot pre-flight).",
    )
    p.add_argument("hostname", nargs="?", default=os.environ.get("SPOT_IP"),
                   help="Spot IP/hostname (default: $SPOT_IP from .env; omit with --dry-run)")
    p.add_argument("--mode", choices=["pose", "turn", "arm"], default="pose",
                   help="pose: body tilt only (default, robot stays put). turn: also rotate in place for large yaw. "
                        "arm: raise the gripper and point it where you look (needs Spot Arm).")
    p.add_argument("--camera", choices=["none", "auto", *SOURCES], default=None,
                   help="show a Spot camera in a window. auto follows head yaw. Default: hand in arm mode, else none. "
                        "Window keys: 1-6 pick camera, 0 auto, r recenter, q quit.")
    p.add_argument("--dry-run", action="store_true", help="print mapped body targets; never talk to Spot")
    p.add_argument("--udp-port", type=int, default=4243, help="sony-head-tracker JSON port (bridge --port + 1)")
    p.add_argument("--rate", type=float, default=20.0, help="command rate, Hz")
    p.add_argument("--body-height", type=float, default=0.0, help="stand height offset, m")
    p.add_argument("--external-estop", action="store_true",
                   help="do not register an e-stop endpoint; rely on the tablet or another client")
    p.add_argument("--no-sit", action="store_true", help="leave Spot standing on exit")
    p.add_argument("--no-recenter", action="store_true", help="don't treat the first sample as straight-ahead")
    p.add_argument("--hand-pos", type=float, nargs=3, metavar=("X", "Y", "Z"), default=(0.6, 0.0, 0.45),
                   help="arm mode: where the gripper hovers, metres in the flat body frame (default 0.6 0 0.45)")

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
    g.add_argument("--invert-yaw", action="store_true", default=d.invert_yaw)
    g.add_argument("--no-invert-pitch", action="store_true", help="tracker pitch is inverted by default")
    g.add_argument("--invert-roll", action="store_true", default=d.invert_roll)
    g.add_argument("--turn-kp", type=float, default=d.turn_kp)
    g.add_argument("--max-turn-rate", type=float, default=d.max_turn_rate_rad_s, help="rad/s")
    return p


def load_env() -> None:
    """Load ``.env`` from the repo root, then map SPOT_* onto the SDK's variables.

    ``bosdyn.client.util.authenticate`` reads BOSDYN_CLIENT_USERNAME/PASSWORD;
    SPOT_USER/SPOT_PASS are friendlier names for the same thing. Existing
    environment variables always win over the file.
    """
    load_dotenv(Path(__file__).resolve().parent.parent / ".env")
    load_dotenv()  # also honour a .env in the current directory
    for src, dst in (("SPOT_USER", "BOSDYN_CLIENT_USERNAME"), ("SPOT_PASS", "BOSDYN_CLIENT_PASSWORD")):
        if src in os.environ and dst not in os.environ:
            os.environ[dst] = os.environ[src]


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
        invert_yaw=args.invert_yaw,
        invert_pitch=not args.no_invert_pitch,
        invert_roll=args.invert_roll,
        turn_kp=args.turn_kp,
        max_turn_rate_rad_s=args.max_turn_rate,
    )


def main(argv: list[str] | None = None) -> int:
    load_env()
    argv = list(sys.argv[1:] if argv is None else argv)
    sub = argv.pop(0) if argv and argv[0] in SUBCOMMANDS else "run"
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    if sub == "viz":
        from .viz import main as viz_main

        vp = argparse.ArgumentParser(prog="eyes-on-me viz", description="Head-tracking gizmo; no robot needed.",
                                     parents=[build_parser()], add_help=False, conflict_handler="resolve")
        vp.add_argument("--arm", action="store_true", help="show the arm envelope instead of the body envelope")
        va = vp.parse_args(argv)
        return viz_main(va.udp_port, cfg_from_args(va), arm=va.arm)

    if sub == "check":
        from .check import main as check_main

        cp = argparse.ArgumentParser(prog="eyes-on-me check", description="Spot pre-flight: network, auth, e-stop, lease, power, arm, cameras.")
        cp.add_argument("hostname", nargs="?", default=os.environ.get("SPOT_IP"))
        cp.add_argument("--udp-port", type=int, default=4243)
        ca = cp.parse_args(argv)
        if not ca.hostname:
            print("error: give a hostname or set SPOT_IP in .env", file=sys.stderr)
            return 2
        return check_main(ca.hostname, ca.udp_port)

    args = build_parser().parse_args(argv)
    if not args.dry_run and not args.hostname:
        print("error: give a hostname or set SPOT_IP in .env (or use --dry-run)", file=sys.stderr)
        return 2

    cfg = cfg_from_args(args)
    camera = args.camera if args.camera is not None else ("hand" if args.mode == "arm" else "none")
    opts = RunOptions(
        mode=args.mode,
        camera=camera,
        rate_hz=args.rate,
        body_height=args.body_height,
        dry_run=args.dry_run,
        external_estop=args.external_estop,
        sit_on_exit=not args.no_sit,
        recenter_on_start=not args.no_recenter,
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

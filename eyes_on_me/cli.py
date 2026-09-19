"""``eyes-on-me`` command line entry point."""

from __future__ import annotations

import argparse
import logging
import sys

from .gaze_mapper import GazeConfig, GazeMapper
from .head_tracker import HeadTrackerReceiver
from .spot_gaze import RunOptions, SpotGaze


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="eyes-on-me",
        description="Point Spot's body where your head is pointing, using Sony headphone head tracking.",
    )
    p.add_argument("hostname", nargs="?", help="Spot IP/hostname (omit with --dry-run)")
    p.add_argument("--mode", choices=["pose", "turn"], default="pose",
                   help="pose: body tilt only (default, robot stays put). turn: also rotate in place for large yaw.")
    p.add_argument("--dry-run", action="store_true", help="print mapped body targets; never talk to Spot")
    p.add_argument("--udp-port", type=int, default=4243, help="sony-head-tracker JSON port (bridge --port + 1)")
    p.add_argument("--rate", type=float, default=20.0, help="command rate, Hz")
    p.add_argument("--body-height", type=float, default=0.0, help="stand height offset, m")
    p.add_argument("--external-estop", action="store_true",
                   help="do not register an e-stop endpoint; rely on the tablet or another client")
    p.add_argument("--no-sit", action="store_true", help="leave Spot standing on exit")
    p.add_argument("--no-recenter", action="store_true", help="don't treat the first sample as straight-ahead")

    g = p.add_argument_group("mapping")
    d = GazeConfig()
    g.add_argument("--max-yaw", type=float, default=d.max_body_yaw_deg, help="deg")
    g.add_argument("--max-pitch", type=float, default=d.max_body_pitch_deg, help="deg")
    g.add_argument("--max-roll", type=float, default=d.max_body_roll_deg, help="deg")
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


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")
    if not args.dry_run and not args.hostname:
        print("error: hostname is required unless --dry-run", file=sys.stderr)
        return 2

    cfg = GazeConfig(
        max_body_yaw_deg=args.max_yaw,
        max_body_pitch_deg=args.max_pitch,
        max_body_roll_deg=args.max_roll,
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
    opts = RunOptions(
        mode=args.mode,
        rate_hz=args.rate,
        body_height=args.body_height,
        dry_run=args.dry_run,
        external_estop=args.external_estop,
        sit_on_exit=not args.no_sit,
        recenter_on_start=not args.no_recenter,
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

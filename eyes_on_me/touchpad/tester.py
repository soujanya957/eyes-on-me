"""``eyes-on-me touchpad-test``: drive the real touchpad, move nothing.

Runs the full chain - hardware signals, decoding, actions - and prints what
Spot *would* be told. No robot, no SDK. Use it to learn the gestures and to
check the mapping before trusting it on a robot that can walk.

``--moving`` pretends a step is always in progress, which is how you exercise
the abort path: a double tap then prints ``stop`` instead of ``gripper_toggle``.
"""

from __future__ import annotations

import time

from . import TouchpadSource
from .gestures import ESTOP, GRIPPER_TOGGLE, STOP, GestureConfig

_ARROWS = {
    "step_forward": "^  forward",
    "step_back": "v  back",
    "step_left": "<  left",
    "step_right": ">  right",
    GRIPPER_TOGGLE: "[] gripper open/close",
    STOP: "!  STOP (abort step)",
    ESTOP: "!! E-STOP (settle then cut)",
}


def main(cfg: GestureConfig, pretend_moving: bool = False) -> int:
    import sys

    # Live tool: piping it must not hold the banner until the process ends.
    sys.stdout.reconfigure(line_buffering=True)
    src = TouchpadSource(cfg).start()
    if not src.remote.available and not src.volume.available:
        print("no touchpad channel available; nothing to test")
        return 2

    print("\nGestures on the RIGHT earcup:")
    print("  swipe forward -> step forward      swipe back -> step back")
    print("  swipe up      -> step left         swipe down -> step right")
    print("  double tap    -> gripper toggle (or STOP while stepping)")
    print("  double tap x5 -> E-STOP")
    print("\n(single tap does nothing on the XM5 - it only beeps)")
    if pretend_moving:
        print("\n--moving: pretending a step is always running, so taps abort")
    print("\nCtrl+C to stop.\n")

    seen: dict[str, int] = {}
    start = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            for action in src.poll(now, moving=pretend_moving):
                seen[action.name] = seen.get(action.name, 0) + 1
                label = _ARROWS.get(action.name, action.name)
                print(f"[{now - start:7.2f}s] {label:<28} (#{seen[action.name]})", flush=True)
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        src.stop()

    print("\n--- summary ---")
    if not seen:
        print("nothing decoded. If the headphones are connected and head tracking")
        print("works, the usual cause is multipoint: disconnect the phone.")
    for name, n in sorted(seen.items(), key=lambda kv: -kv[1]):
        print(f"  {_ARROWS.get(name, name):<28} {n:>3}")
    return 0

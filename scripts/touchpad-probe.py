#!/usr/bin/env python3
"""Print what the WH-1000XM5 touchpad actually sends to this Mac.

Read-only diagnostic for step 2/3 of touchpad-controls/PLAN.md: no robot, no
Spot SDK. Do each gesture a few times and watch what shows up.

Two channels are watched, because it is not obvious up front which one a
given gesture uses:

* **media keys** - the headphones turn tap / double tap / swipe forward-back
  into AVRCP transport commands, which macOS delivers as ``NX_SYSDEFINED``
  events. Every keycode is logged, not just the ones the plan predicts.
* **output volume** - swipe up/down may instead drive AVRCP absolute volume,
  which changes the device volume without producing any key event. Polled,
  and the volume is put back so repeated swipes keep working.

Gestures with no host-visible effect (press-and-hold goes to Siri, and
"cover the cup" quick-attention is handled inside the headphones) will simply
never print. That absence is the result, not a failure of this script.

Needs Accessibility permission for the event tap. Ctrl+C prints a summary.
"""

from __future__ import annotations

import collections
import subprocess
import sys
import threading
import time

import Quartz
from AppKit import NSEvent

# NX_KEYTYPE_* codes carried in an NX_SYSDEFINED subtype-8 event.
KEYCODES = {
    0: "SOUND_UP", 1: "SOUND_DOWN", 2: "BRIGHTNESS_UP", 3: "BRIGHTNESS_DOWN",
    7: "MUTE", 16: "PLAY", 17: "NEXT", 18: "PREVIOUS", 19: "FAST", 20: "REWIND",
}
# What PLAN.md expects each signal to mean on the XM5 touchpad.
MEANING = {
    "PLAY": "tap",
    "NEXT": "double tap / swipe forward",
    "PREVIOUS": "triple tap / swipe back",
    "FAST": "swipe forward (held)",
    "REWIND": "swipe back (held)",
    "SOUND_UP": "swipe up",
    "SOUND_DOWN": "swipe down",
}

NX_SYSDEFINED = 14
counts: collections.Counter[str] = collections.Counter()
_start = time.monotonic()
_lock = threading.Lock()


def report(signal: str, detail: str = "") -> None:
    with _lock:
        counts[signal] += 1
        n = counts[signal]
    meaning = MEANING.get(signal, "")
    note = f"  <- {meaning}" if meaning else ""
    print(f"[{time.monotonic() - _start:7.2f}s] {signal:<12} #{n:<3}{detail}{note}", flush=True)


# --------------------------------------------------------------- media keys
def tap_callback(proxy, type_, event, refcon):
    if type_ in (Quartz.kCGEventTapDisabledByTimeout, Quartz.kCGEventTapDisabledByUserInput):
        Quartz.CGEventTapEnable(refcon[0], True)  # re-arm and swallow nothing
        return event
    try:
        ns = NSEvent.eventWithCGEvent_(event)
        if ns is None or ns.subtype() != 8:
            return event
        data1 = ns.data1()
        keycode = (data1 >> 16) & 0xFFFF
        pressed = ((data1 & 0xFF00) >> 8) == 0x0A
    except Exception:
        return event
    if not pressed:
        return event  # ignore key-up, one line per gesture
    name = KEYCODES.get(keycode, f"UNKNOWN({keycode})")
    report(name, f" keycode={keycode}")
    return None  # swallow, so Music/Spotify do not react while probing


def start_event_tap() -> bool:
    holder: list = [None]
    tap = Quartz.CGEventTapCreate(
        Quartz.kCGSessionEventTap,
        Quartz.kCGHeadInsertEventTap,
        Quartz.kCGEventTapOptionDefault,
        Quartz.CGEventMaskBit(NX_SYSDEFINED),
        tap_callback,
        holder,
    )
    if tap is None:
        print("!! Could not create the event tap - grant Accessibility permission:", file=sys.stderr)
        print("   System Settings > Privacy & Security > Accessibility", file=sys.stderr)
        print("   Tap/swipe-forward/back will not be detected. Volume swipes still will.\n", file=sys.stderr)
        return False
    holder[0] = tap
    source = Quartz.CFMachPortCreateRunLoopSource(None, tap, 0)

    def run():
        Quartz.CFRunLoopAddSource(Quartz.CFRunLoopGetCurrent(), source, Quartz.kCFRunLoopCommonModes)
        Quartz.CGEventTapEnable(tap, True)
        Quartz.CFRunLoopRun()

    threading.Thread(target=run, daemon=True, name="event-tap").start()
    return True


# ------------------------------------------------------------------ volume
def get_volume() -> int | None:
    try:
        out = subprocess.run(["osascript", "-e", "output volume of (get volume settings)"],
                             capture_output=True, text=True, timeout=2)
        return int(out.stdout.strip())
    except Exception:
        return None


def set_volume(level: int) -> None:
    try:
        subprocess.run(["osascript", "-e", f"set volume output volume {level}"], timeout=2)
    except Exception:
        pass


def watch_volume(baseline: int, stop: threading.Event) -> None:
    """Poll for AVRCP absolute-volume changes and restore the baseline.

    Resetting after each change keeps headroom in both directions, so you can
    swipe up ten times in a row without pinning at 100.
    """
    last = baseline
    while not stop.is_set():
        time.sleep(0.12)
        v = get_volume()
        if v is None or v == last:
            continue
        if v > last:
            report("SOUND_UP", f" volume {last}->{v}")
        else:
            report("SOUND_DOWN", f" volume {last}->{v}")
        set_volume(baseline)
        last = baseline


def main() -> int:
    # Line-buffer: this is a live diagnostic, and piping it must not hide output
    # until the process ends (it is normally ended with Ctrl+C).
    sys.stdout.reconfigure(line_buffering=True)
    sys.stderr.reconfigure(line_buffering=True)
    print(__doc__.split("\n\n")[0])
    print("\nTry each gesture on the RIGHT earcup, a few times each:")
    print("  tap  |  double tap  |  triple tap  |  swipe forward / back")
    print("  swipe up / down  |  press and hold  |  cover the whole cup")
    print("\nAnything that prints nothing is not visible to the Mac. Ctrl+C for a summary.\n")

    tapped = start_event_tap()
    baseline = get_volume()
    stop = threading.Event()
    if baseline is None:
        print("!! Could not read output volume; swipe up/down will not be detected.\n", file=sys.stderr)
    else:
        if not 10 <= baseline <= 90:  # leave room to move in both directions
            baseline = 50
            set_volume(baseline)
        print(f"volume baseline {baseline} (restored on exit)\n")
        threading.Thread(target=watch_volume, args=(baseline, stop), daemon=True, name="volume").start()

    if not tapped and baseline is None:
        print("Neither channel is available; nothing to probe.", file=sys.stderr)
        return 2

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        if baseline is not None:
            set_volume(baseline)

    print("\n\n--- summary ---")
    if not counts:
        print("nothing detected at all.")
        if not tapped:
            print("the event tap never started; grant Accessibility and rerun.")
    for signal, n in counts.most_common():
        print(f"  {signal:<12} {n:>3}   {MEANING.get(signal, '')}")
    seen = set(counts)
    missing = [s for s in ("PLAY", "NEXT", "PREVIOUS", "SOUND_UP", "SOUND_DOWN") if s not in seen]
    if missing:
        print(f"\n  not seen: {', '.join(missing)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

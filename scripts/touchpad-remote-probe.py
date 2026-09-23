#!/usr/bin/env python3
"""Can we receive the XM5 touchpad's transport commands at all?

Follow-up to ``touchpad-probe.py``, which established that swipe up/down
arrive as output-volume changes while tap / double tap / swipe forward-back
do **not** arrive as ``NX_SYSDEFINED`` media keys. Those gestures do reach
macOS - a double tap pauses the Now Playing app - so they travel through
MediaRemote rather than the event stream a ``CGEventTap`` can see.

This probes the supported way in: register handlers on
``MPRemoteCommandCenter`` and declare ourselves playing via
``MPNowPlayingInfoCenter``. If macOS makes this process the Now Playing
target, the headphones' play/pause/next/previous land in these handlers.

Nothing is sent to Spot. Ctrl+C to stop.

Caveat: eligibility for Now Playing is an OS decision. A plain interpreter
may be refused where a bundled .app would not, so a silent run here is a
result worth having - it tells us a bundle is required.
"""

from __future__ import annotations

import signal
import sys
import time

from AppKit import NSApplication, NSApplicationActivationPolicyAccessory
from MediaPlayer import (
    MPMusicPlaybackStatePlaying,
    MPNowPlayingInfoCenter,
    MPNowPlayingInfoPropertyElapsedPlaybackTime,
    MPNowPlayingInfoPropertyPlaybackRate,
    MPMediaItemPropertyTitle,
    MPMediaItemPropertyPlaybackDuration,
    MPRemoteCommandCenter,
    MPRemoteCommandHandlerStatusSuccess,
)

_start = time.monotonic()
_seen: dict[str, int] = {}


def note(name: str) -> None:
    _seen[name] = _seen.get(name, 0) + 1
    print(f"[{time.monotonic() - _start:7.2f}s] {name:<16} #{_seen[name]}", flush=True)


def handler(name):
    def cb(event):
        note(name)
        return MPRemoteCommandHandlerStatusSuccess
    return cb


def main() -> int:
    sys.stdout.reconfigure(line_buffering=True)
    app = NSApplication.sharedApplication()
    app.setActivationPolicy_(NSApplicationActivationPolicyAccessory)

    cc = MPRemoteCommandCenter.sharedCommandCenter()
    commands = {
        "play": cc.playCommand(),
        "pause": cc.pauseCommand(),
        "togglePlayPause": cc.togglePlayPauseCommand(),
        "nextTrack": cc.nextTrackCommand(),
        "previousTrack": cc.previousTrackCommand(),
        "seekForward": cc.seekForwardCommand(),
        "seekBackward": cc.seekBackwardCommand(),
    }
    for name, cmd in commands.items():
        cmd.setEnabled_(True)
        cmd.addTargetWithHandler_(handler(name))

    # Declare ourselves as actively playing, otherwise macOS has no reason to
    # route transport commands here.
    info = {
        MPMediaItemPropertyTitle: "eyes-on-me touchpad probe",
        MPMediaItemPropertyPlaybackDuration: 3600.0,
        MPNowPlayingInfoPropertyElapsedPlaybackTime: 0.0,
        MPNowPlayingInfoPropertyPlaybackRate: 1.0,
    }
    centre = MPNowPlayingInfoCenter.defaultCenter()
    centre.setNowPlayingInfo_(info)
    centre.setPlaybackState_(MPMusicPlaybackStatePlaying)

    print(__doc__.split("\n\n")[0])
    print("\nRegistered:", ", ".join(commands))
    print("Now do on the RIGHT earcup: double tap, swipe forward, swipe back.")
    print("(single tap only beeps on the XM5 - it is not a transport command)")
    print("\nNothing printing means macOS did not route commands here.\n")

    signal.signal(signal.SIGINT, lambda *_: (_summary(), sys.exit(0)))
    app.run()
    return 0


def _summary() -> None:
    print("\n--- summary ---")
    if not _seen:
        print("no remote commands received; this process was never made the")
        print("Now Playing target. A bundled .app is likely required.")
    for name, n in sorted(_seen.items(), key=lambda kv: -kv[1]):
        print(f"  {name:<16} {n:>3}")


if __name__ == "__main__":
    raise SystemExit(main())

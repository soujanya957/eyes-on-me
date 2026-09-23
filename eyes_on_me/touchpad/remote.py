"""Double tap and swipe forward/back, via MediaRemote.

These gestures reach macOS as AVRCP transport commands and are routed to
whatever app currently holds "Now Playing". They are **not** delivered as
``NX_SYSDEFINED`` media keys, so a ``CGEventTap`` never sees them no matter
what permissions it has - measured, see ``touchpad-controls/PLAN.md`` section 0.

So we become the Now Playing app: register handlers on
``MPRemoteCommandCenter`` and declare ourselves playing. No Accessibility
permission and no ``.app`` bundle are needed.

Callbacks are delivered on the main run loop, which the control loop owns, so
:meth:`RemoteCommands.poll` gives that run loop a zero-length slice each tick
and then drains what arrived - the same shape as ``CameraViewer.pump``.
"""

from __future__ import annotations

import logging
import queue

from .gestures import NEXT, PREV, TAP, Signal

log = logging.getLogger("eyes_on_me.touchpad.remote")

try:
    from AppKit import NSApplication, NSApplicationActivationPolicyAccessory
    from CoreFoundation import CFRunLoopRunInMode, kCFRunLoopDefaultMode
    from MediaPlayer import (
        MPMediaItemPropertyPlaybackDuration,
        MPMediaItemPropertyTitle,
        MPMusicPlaybackStatePlaying,
        MPNowPlayingInfoCenter,
        MPNowPlayingInfoPropertyElapsedPlaybackTime,
        MPNowPlayingInfoPropertyPlaybackRate,
        MPRemoteCommandCenter,
        MPRemoteCommandHandlerStatusSuccess,
    )

    _HAVE_MEDIAPLAYER = True
except ImportError:  # pragma: no cover - darwin only
    _HAVE_MEDIAPLAYER = False

# Every transport command the touchpad can produce, mapped to our signal kinds.
# play/pause/toggle all come from the same double tap; which one arrives
# depends on the playback state macOS thinks we are in, so they are one signal.
_COMMANDS = {
    "playCommand": TAP,
    "pauseCommand": TAP,
    "togglePlayPauseCommand": TAP,
    "nextTrackCommand": NEXT,
    "previousTrackCommand": PREV,
}


class RemoteCommands:
    """Receives touchpad transport commands as :class:`Signal` objects."""

    def __init__(self):
        self._q: queue.Queue[Signal] = queue.Queue()
        self._centre = None
        self.available = False

    def start(self) -> "RemoteCommands":
        if not _HAVE_MEDIAPLAYER:
            log.warning("MediaPlayer unavailable; double tap and swipe fwd/back will not be seen")
            return self
        import time

        app = NSApplication.sharedApplication()
        app.setActivationPolicy_(NSApplicationActivationPolicyAccessory)

        cc = MPRemoteCommandCenter.sharedCommandCenter()
        for attr, kind in _COMMANDS.items():
            cmd = getattr(cc, attr)()
            cmd.setEnabled_(True)
            cmd.addTargetWithHandler_(self._handler(kind, time.monotonic))

        # Without claiming Now Playing, macOS has no reason to route here.
        self._centre = MPNowPlayingInfoCenter.defaultCenter()
        self._claim()
        self.available = True
        return self

    def _claim(self) -> None:
        self._centre.setNowPlayingInfo_({
            MPMediaItemPropertyTitle: "eyes-on-me",
            MPMediaItemPropertyPlaybackDuration: 3600.0,
            MPNowPlayingInfoPropertyElapsedPlaybackTime: 0.0,
            MPNowPlayingInfoPropertyPlaybackRate: 1.0,
        })
        self._centre.setPlaybackState_(MPMusicPlaybackStatePlaying)

    def _handler(self, kind: str, clock):
        def cb(event):
            self._q.put(Signal(kind, clock()))
            return MPRemoteCommandHandlerStatusSuccess
        return cb

    def poll(self) -> list[Signal]:
        """Service the run loop, then return whatever gestures arrived."""
        if not self.available:
            return []
        CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.0, False)
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                return out

    def stop(self) -> None:
        if self.available and self._centre is not None:
            self._centre.setNowPlayingInfo_(None)
        self.available = False

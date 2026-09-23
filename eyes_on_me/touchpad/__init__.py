"""Touchpad control: WH-1000XM5 earcup gestures as a five-button controller.

``remote`` and ``volume`` turn hardware gestures into :class:`~.gestures.Signal`
objects, ``gestures`` decodes those into :class:`~.gestures.Action` objects
(pure, unit-tested), and ``actions`` runs them on Spot. The control loop owns
the timing: it calls :meth:`TouchpadSource.poll` once per tick, exactly where
it already polls the camera window and stdin.
"""

from __future__ import annotations

import logging

from .gestures import Action, GestureConfig, GestureDecoder, Signal
from .remote import RemoteCommands
from .volume import VolumeWatcher

log = logging.getLogger("eyes_on_me.touchpad")

__all__ = ["Action", "GestureConfig", "GestureDecoder", "Signal", "TouchpadSource"]


class TouchpadSource:
    """Both signal channels plus the decoder, polled as one thing."""

    def __init__(self, cfg: GestureConfig | None = None):
        self.decoder = GestureDecoder(cfg or GestureConfig())
        self.remote = RemoteCommands()
        self.volume = VolumeWatcher()

    def start(self) -> "TouchpadSource":
        self.remote.start()
        self.volume.start()
        if not self.remote.available and not self.volume.available:
            log.error("touchpad: neither channel started; gestures will do nothing")
        elif not self.remote.available:
            log.warning("touchpad: no transport commands (double tap, swipe fwd/back inactive)")
        elif not self.volume.available:
            log.warning("touchpad: no volume channel (swipe up/down inactive)")
        else:
            log.info("touchpad: ready (swipe fwd/back/up/down, double tap)")
            log.info("touchpad: if nothing responds, disconnect the phone - "
                     "with multipoint the headset sends gestures there, not here")
        return self

    def poll(self, now: float, moving: bool) -> list[Action]:
        """Signals since the last tick, decoded. Safe to call at loop rate."""
        actions = []
        signals = self.remote.poll()
        v = self.volume.poll(now)
        if v is not None:
            signals.append(v)
        for sig in signals:
            action = self.decoder.feed(sig, moving)
            if action is not None:
                actions.append(action)
        deferred = self.decoder.due(now)
        if deferred is not None:
            actions.append(deferred)
        return actions

    def stop(self) -> None:
        self.remote.stop()
        self.volume.stop()

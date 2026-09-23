"""Swipe up / swipe down, read as output-volume steps.

The XM5 sends swipe up/down as AVRCP absolute-volume changes, so they never
appear as key events - the output device's volume simply moves. We watch that
value, turn each change into a signal, and put it straight back, so the user
can swipe the same way ten times without pinning at 0 or 100.

Reading the volume is a cheap synchronous CoreAudio call, so this polls from
the control loop's own tick instead of installing a listener block: no extra
thread, no dispatch queue, and nothing to tear down. It is deliberately not
``osascript``, which would mean spawning a subprocess many times a second
inside the process that is also driving a robot.

Two things about the CoreAudio binding that are easy to get wrong, both
found the hard way:

* the qualifier argument must be ``b""``; passing ``None`` raises
  ``TypeError: converting to a C array``;
* Bluetooth output devices commonly do **not** implement volume on the main
  element - it returns ``kAudioHardwareUnknownPropertyError`` (``'who?'``,
  2003332927) - and expose per-channel volume on elements 1 and 2 instead.
  So the usable elements are discovered at startup rather than assumed.
"""

from __future__ import annotations

import logging
import struct

from .gestures import VOL_DOWN, VOL_UP, Signal

log = logging.getLogger("eyes_on_me.touchpad.volume")

try:
    import CoreAudio as CA

    _HAVE_COREAUDIO = True
except ImportError:  # pragma: no cover - darwin only
    _HAVE_COREAUDIO = False

_OK = 0
#: Main element first; stereo channels as the fallback most Bluetooth devices need.
_ELEMENT_SETS = ((0,), (1, 2))


def _address(selector, scope, element):
    a = CA.AudioObjectPropertyAddress()
    a.mSelector = selector
    a.mScope = scope
    a.mElement = element
    return a


def _default_output_device() -> int | None:
    addr = _address(CA.kAudioHardwarePropertyDefaultOutputDevice,
                    CA.kAudioObjectPropertyScopeGlobal, CA.kAudioObjectPropertyElementMain)
    try:
        err, _, data = CA.AudioObjectGetPropertyData(
            CA.kAudioObjectSystemObject, addr, 0, b"", 4, bytearray(4))
    except Exception as exc:  # pragma: no cover - API shape varies by version
        log.debug("default output query failed: %s", exc)
        return None
    if err != _OK:
        return None
    return struct.unpack("<I", bytes(data))[0]


class VolumeWatcher:
    """Polls the default output device's volume; each change is one swipe."""

    #: Volume is 0..1 and one XM5 step is ~1/16, so this sits well below a step.
    THRESHOLD = 0.01

    def __init__(self):
        self._device: int | None = None
        self._elements: tuple[int, ...] = ()
        self._baseline = 0.5
        self._original: float | None = None
        self.available = False

    # ----------------------------------------------------------------- setup
    def start(self) -> "VolumeWatcher":
        if not _HAVE_COREAUDIO:
            log.warning("CoreAudio unavailable; swipe up/down will not be seen")
            return self
        self._device = _default_output_device()
        if self._device is None:
            log.warning("no default output device; swipe up/down will not be seen")
            return self

        for elements in _ELEMENT_SETS:
            if self._read_element(elements[0]) is not None:
                self._elements = elements
                break
        else:
            log.warning("output device exposes no readable volume; swipe up/down will not be seen")
            return self

        current = self._read()
        self._original = current
        # Park mid-range so there is room to move in both directions.
        self._baseline = current if 0.15 <= current <= 0.85 else 0.5
        self._write(self._baseline)
        self.available = True
        log.debug("volume watcher on device %s elements %s", self._device, self._elements)
        return self

    # ------------------------------------------------------------------ i/o
    def _read_element(self, element: int) -> float | None:
        addr = _address(CA.kAudioDevicePropertyVolumeScalar,
                        CA.kAudioObjectPropertyScopeOutput, element)
        try:
            err, _, data = CA.AudioObjectGetPropertyData(self._device, addr, 0, b"", 4, bytearray(4))
        except Exception:
            return None
        if err != _OK:
            return None
        return struct.unpack("<f", bytes(data))[0]

    def _read(self) -> float:
        """Loudest channel: either channel moving means the user swiped."""
        values = [v for v in (self._read_element(e) for e in self._elements) if v is not None]
        return max(values) if values else self._baseline

    def _write(self, value: float) -> None:
        value = max(0.0, min(1.0, value))
        payload = struct.pack("<f", value)
        for element in self._elements:
            addr = _address(CA.kAudioDevicePropertyVolumeScalar,
                            CA.kAudioObjectPropertyScopeOutput, element)
            try:
                CA.AudioObjectSetPropertyData(self._device, addr, 0, b"", 4, payload)
            except Exception as exc:  # pragma: no cover
                log.debug("volume write failed: %s", exc)

    # ----------------------------------------------------------------- loop
    def poll(self, now: float) -> Signal | None:
        """One signal if the volume moved since we last parked it."""
        if not self.available:
            return None
        delta = self._read() - self._baseline
        if abs(delta) < self.THRESHOLD:
            return None
        self._write(self._baseline)
        return Signal(VOL_UP if delta > 0 else VOL_DOWN, now)

    def stop(self) -> None:
        """Give the user their volume back."""
        if self.available and self._original is not None:
            self._write(self._original)
        self.available = False

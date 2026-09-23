"""Pure gesture decoding: a stream of touchpad signals becomes robot actions.

No SDK, no macOS frameworks, no clock of its own - every entry point takes an
explicit timestamp - so the whole mapping is unit-testable without headphones
or a robot. See ``touchpad-controls/PLAN.md`` for what the hardware actually
emits (measured, not assumed).

The five signals the WH-1000XM5 can deliver, and what they do here:

===========  ==============  ==================================================
gesture      signal          action
===========  ==============  ==================================================
swipe fwd    ``next``        one step forward
swipe back   ``prev``        one step back
swipe up     ``vol_up``      one step left
swipe down   ``vol_down``    one step right
double tap   ``tap``         **stepping**: abort now. **idle**: toggle gripper.
double tap   ``tap`` x5      e-stop (settle, then cut)
===========  ==============  ==================================================

Why a double tap means two different things: stopping has to be the fastest
path on the controller (PLAN.md section 5, rule 3), and the touchpad has no
signal left to spare. So a tap is read against what the robot is doing -
identical to how the original plan read a single tap. While a step is running
the tap aborts it on the same tick, with no waiting for a longer sequence.
Idle, there is nothing to stop, so the tap is free to mean gripper.

That leaves one unavoidable cost: an idle tap cannot fire the gripper
immediately, because it might turn out to be the first of the five that mean
e-stop. Idle taps are therefore held for ``tap_settle_s`` and only then become
a gripper toggle. An aborting tap is never delayed, which is the case that
matters.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# Signal kinds, as produced by touchpad.remote and touchpad.volume.
TAP = "tap"
NEXT = "next"
PREV = "prev"
VOL_UP = "vol_up"
VOL_DOWN = "vol_down"

# Action names consumed by touchpad.actions.
STEP_FORWARD = "step_forward"
STEP_BACK = "step_back"
STEP_LEFT = "step_left"
STEP_RIGHT = "step_right"
GRIPPER_TOGGLE = "gripper_toggle"
STOP = "stop"
ESTOP = "estop"

_DIRECTIONS = {
    NEXT: STEP_FORWARD,
    PREV: STEP_BACK,
    VOL_UP: STEP_LEFT,
    VOL_DOWN: STEP_RIGHT,
}


@dataclass(frozen=True)
class Signal:
    kind: str
    t: float


@dataclass(frozen=True)
class Action:
    name: str


@dataclass
class GestureConfig:
    #: Repeats of the same direction closer together than this are one gesture.
    #: The XM5 repeats volume steps about every 200 ms when held at the edge,
    #: and each repeat is a deliberate extra step, so this stays short.
    debounce_s: float = 0.15
    #: How long an idle tap waits to see whether more taps follow.
    tap_settle_s: float = 0.40
    #: Taps within this window count toward the e-stop sequence.
    estop_window_s: float = 2.0
    #: Taps required for e-stop.
    estop_taps: int = 5


@dataclass
class GestureDecoder:
    """Signals in, actions out. Feed it, then poll :meth:`due` on every tick."""

    cfg: GestureConfig = field(default_factory=GestureConfig)
    _last_direction: dict[str, float] = field(default_factory=dict, init=False)
    _taps: list[float] = field(default_factory=list, init=False)
    _pending_tap_at: float | None = field(default=None, init=False)

    def feed(self, sig: Signal, moving: bool) -> Action | None:
        """Handle one signal. Returns an action to run immediately, if any."""
        if sig.kind in _DIRECTIONS:
            last = self._last_direction.get(sig.kind)
            if last is not None and sig.t - last < self.cfg.debounce_s:
                return None
            self._last_direction[sig.kind] = sig.t
            return Action(_DIRECTIONS[sig.kind])

        if sig.kind != TAP:
            return None

        # Keep only taps still inside the e-stop window.
        self._taps = [t for t in self._taps if sig.t - t < self.cfg.estop_window_s]
        self._taps.append(sig.t)
        if len(self._taps) >= self.cfg.estop_taps:
            self._taps.clear()
            self._pending_tap_at = None
            return Action(ESTOP)

        if moving:
            # Stopping never waits and never counts toward anything else.
            self._pending_tap_at = None
            return Action(STOP)

        # Idle: hold it, in case this is the start of an e-stop sequence.
        self._pending_tap_at = sig.t
        return None

    def due(self, now: float) -> Action | None:
        """Actions that were deferred: call once per control-loop tick."""
        at = self._pending_tap_at
        if at is not None and now - at >= self.cfg.tap_settle_s:
            self._pending_tap_at = None
            return Action(GRIPPER_TOGGLE)
        return None

from eyes_on_me.touchpad.gestures import (
    ESTOP,
    GRIPPER_TOGGLE,
    NEXT,
    PREV,
    STEP_BACK,
    STEP_FORWARD,
    STEP_LEFT,
    STEP_RIGHT,
    STOP,
    TAP,
    VOL_DOWN,
    VOL_UP,
    GestureConfig,
    GestureDecoder,
    Signal,
)


def decoder(**kw):
    return GestureDecoder(GestureConfig(**kw))


def names(actions):
    return [a.name for a in actions if a is not None]


def test_each_swipe_steps_one_way():
    d = decoder()
    got = [d.feed(Signal(kind, t), moving=False) for t, kind in
           enumerate((NEXT, PREV, VOL_UP, VOL_DOWN))]
    assert names(got) == [STEP_FORWARD, STEP_BACK, STEP_LEFT, STEP_RIGHT]


def test_repeats_within_debounce_collapse():
    d = decoder(debounce_s=0.15)
    assert d.feed(Signal(NEXT, 0.00), moving=False).name == STEP_FORWARD
    assert d.feed(Signal(NEXT, 0.10), moving=False) is None      # too soon
    assert d.feed(Signal(NEXT, 0.30), moving=False).name == STEP_FORWARD


def test_debounce_is_per_direction():
    """Swiping up then forward in quick succession is two deliberate steps."""
    d = decoder(debounce_s=0.15)
    assert d.feed(Signal(VOL_UP, 0.00), moving=False).name == STEP_LEFT
    assert d.feed(Signal(NEXT, 0.05), moving=False).name == STEP_FORWARD


def test_tap_while_stepping_stops_immediately():
    d = decoder()
    assert d.feed(Signal(TAP, 0.0), moving=True).name == STOP
    assert d.due(0.0) is None          # nothing deferred
    assert d.due(10.0) is None         # and no late gripper toggle either


def test_idle_tap_becomes_gripper_after_settling():
    d = decoder(tap_settle_s=0.4)
    assert d.feed(Signal(TAP, 0.0), moving=False) is None   # held, not immediate
    assert d.due(0.2) is None                               # still settling
    assert d.due(0.5).name == GRIPPER_TOGGLE
    assert d.due(0.9) is None                               # fires once only


def test_five_taps_estop_and_suppress_the_gripper():
    d = decoder(tap_settle_s=0.4, estop_taps=5, estop_window_s=2.0)
    got = [d.feed(Signal(TAP, 0.1 * i), moving=False) for i in range(5)]
    assert names(got) == [ESTOP]
    assert d.due(10.0) is None, "a pending gripper toggle must not survive an e-stop"


def test_estop_sequence_works_while_moving_too():
    """Each tap stops, but five in the window still escalate to e-stop."""
    d = decoder(estop_taps=5, estop_window_s=2.0)
    got = [d.feed(Signal(TAP, 0.1 * i), moving=True) for i in range(5)]
    assert names(got) == [STOP, STOP, STOP, STOP, ESTOP]


def test_stale_taps_do_not_count_toward_estop():
    d = decoder(estop_taps=5, estop_window_s=2.0, tap_settle_s=0.4)
    for i in range(4):
        d.feed(Signal(TAP, 0.1 * i), moving=False)
    # Well outside the window: this is a fresh first tap, not the fifth.
    assert d.feed(Signal(TAP, 9.0), moving=False) is None
    assert d.due(9.5).name == GRIPPER_TOGGLE


def test_unknown_signal_is_ignored():
    assert decoder().feed(Signal("wat", 0.0), moving=False) is None

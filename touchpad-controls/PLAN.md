# Touchpad controls — build plan

Use the WH-1000XM5's right-earcup touchpad as a five-button controller for
Spot, layered on top of head tracking: **head = where to go, touchpad = go /
stop / posture**. Status: planned, not started.

## 1. What the touchpad can give us

The headphones decide the gesture in firmware and send a standard Bluetooth
AVRCP command. macOS turns those into exactly five observable signals:

| gesture                         | signal      | how it reaches us                                   |
| ------------------------------- | ----------- | --------------------------------------------------- |
| tap                             | `play`      | `NX_SYSDEFINED` media-key event, keycode 16         |
| double tap **or** swipe forward | `next`      | media-key event, keycode 17 (or 19 `FAST`)          |
| triple tap **or** swipe back    | `prev`      | media-key event, keycode 18 (or 20 `REWIND`)        |
| swipe up                        | `vol_up`    | output-device volume increases one step (no key)    |
| swipe down                      | `vol_down`  | output-device volume decreases one step             |
| press and hold                  | —           | Siri; not interceptable, ignore                     |

Hard limits that shape the design:

- Double-tap and swipe-forward are the same signal. Same for triple/back.
- Five discrete buttons, no analog, no hold. Latency ~100–300 ms.
- Nothing is safe to bind to a single accidental touch that moves the robot
  unboundedly.

## 2. Mapping (v1 default, all rebindable)

| signal                | action                                                                          |
| --------------------- | ------------------------------------------------------------------------------- |
| `vol_up`              | walk **forward** 0.4 m/s for 2.0 s, heading = current body heading; repeat to extend |
| `vol_down`            | walk **backward** 0.3 m/s for 2.0 s                                             |
| `next`                | strafe **right** 0.4 m/s for 1.2 s                                              |
| `prev`                | strafe **left** 0.4 m/s for 1.2 s                                               |
| `play`                | **stop**: cancel any motion. If already stopped: recenter head tracking          |
| `play` ×3 in 1.5 s    | sit ⇄ stand toggle                                                              |
| `play` ×5 in 2.0 s    | e-stop, settle-then-cut                                                         |

Every motion is a fixed-duration velocity command that expires on its own
(dead-man built into the robot: `end_time_secs`). A stray swipe moves Spot
at most ~0.8 m. Steering stays with the head: in `--mode turn` Spot's
heading already follows head yaw, so "swipe up" walks where you're looking.

Alternate profile `--touchpad-profile camera`: `next`/`prev` cycle the
camera window instead of strafing. Profiles are just dicts.

## 3. Architecture

```
touchpad/
  events.py      Quartz event tap  -> Signal(kind, t)   (play/next/prev)
  volume.py      CoreAudio listener -> Signal(kind, t)   (vol_up/vol_down)
  gestures.py    pure: Signal stream -> Action (debounce, ×3/×5 sequences)   [unit-testable]
  actions.py     Action -> Spot command (walk/strafe/stop/sit/estop/recenter)
```

Wire-in: `SpotGaze.run()` polls a `queue.Queue[Action]` once per tick, same
place it polls `StdinKeys` and the camera window. No new threads in the
control loop; the tap and the audio listener each own a background thread.

### 3.1 `events.py` — media keys

- `CGEventTapCreate(kCGSessionEventTap, kCGHeadInsertEventTap,
  kCGEventTapOptionDefault, CGEventMaskBit(NX_SYSDEFINED)=1<<14, cb)`.
- In `cb`: `NSEvent.eventWithCGEvent_`, require `subtype() == 8`, decode
  `data1`: keycode `= (data1 >> 16) & 0xFFFF`, key-down when
  `((data1 & 0xFF00) >> 8) == 0x0A`. Emit on key-down only.
- Return `None` to swallow so Music/Spotify never see it; return the event
  unchanged for everything else.
- Own `CFRunLoop` on a daemon thread; `CGEventTapEnable(tap, True)` and
  re-enable on `kCGEventTapDisabledByTimeout`.
- Permission: Accessibility (System Settings → Privacy & Security). If
  `CGEventTapCreate` returns `None`, log the exact settings pane and continue
  without touchpad rather than crash.

### 3.2 `volume.py` — swipe up/down

- Find the XM5 output device: `AudioObjectGetPropertyData` on
  `kAudioHardwarePropertyDefaultOutputDevice`; verify its name contains
  "WH-1000XM5" (else warn: swipes will be lost).
- Read `kAudioDevicePropertyVolumeScalar` (main element, output scope) as
  baseline; if baseline is 0.0 or 1.0, set to 0.5 first so both directions
  have room.
- `AudioObjectAddPropertyListenerBlock` on that property. On change:
  `delta = new - baseline`; `|delta| > 0.02` → emit `vol_up`/`vol_down`;
  then write the baseline back. Ignore the change event our own write
  causes (flag + 50 ms window).
- Restore the user's original volume on exit (`finally:`).
- Hold-at-edge on the XM5 sends repeated steps ~every 200 ms; the gesture
  layer treats repeats within 300 ms as one "extend walk" each, which gives
  a natural hold-to-keep-walking feel.

### 3.3 `gestures.py` — pure, testable

```python
@dataclass(frozen=True)
class Signal: kind: str; t: float          # play/next/prev/vol_up/vol_down
@dataclass(frozen=True)
class Action: name: str                    # walk_fwd/walk_back/strafe_l/strafe_r/stop/recenter/sit_stand/estop

class GestureDecoder:
    def feed(self, s: Signal, moving: bool) -> Action | None
```

- `play`: push timestamp; if 5 in 2.0 s → `estop`, elif 3 in 1.5 s →
  `sit_stand`, else schedule a single-`play` action 0.35 s later unless
  more taps arrive (so a ×3 doesn't first fire a stop). Exception: if
  `moving` is true, fire `stop` immediately on the first tap — stopping must
  never wait.
- `next`/`prev`/`vol_*`: debounce 150 ms, then map through the profile.
- Tests: tap-while-moving stops immediately; ×3 yields exactly one
  `sit_stand` and no `stop`/`recenter`; ×5 yields `estop`; volume repeats
  collapse to one action per step; stale taps outside the window don't
  count.

### 3.4 `actions.py` — Spot side

| action      | SDK call                                                                              |
| ----------- | ------------------------------------------------------------------------------------- |
| walk_fwd    | `synchro_velocity_command(v_x=0.4, v_y=0, v_rot=<from turn mode or 0>)`, `end_time_secs=now+2.0` |
| walk_back   | same, `v_x=-0.3`                                                                      |
| strafe_l/r  | `v_y=±0.4`, 1.2 s                                                                     |
| stop        | `synchro_stand_command(footprint_R_body=<current head pose>)` — overrides the velocity command |
| recenter    | `mapper.recenter(...)` + `_heading0 = odom yaw` (existing code path)                   |
| sit_stand   | if standing: `synchro_sit_command()`; else `blocking_stand()`; head tracking pauses while sitting |
| estop       | `SpotGaze.estop(immediate=False)` (existing)                                           |

Interaction with the existing per-tick stand command: while a walk is
active (`until > now`), the loop sends the velocity command *with*
`mobility_params(footprint_R_body=…)` so the body still follows the head,
exactly as turn mode does; when it expires, the loop falls back to the
stand command. In `arm` mode, walking is disabled (arm raised + walking is
a different safety envelope); only stop/recenter/sit_stand/estop apply.

## 4. CLI

```
--touchpad                     enable (default off)
--touchpad-profile {drive,camera}
--walk-speed 0.4  --walk-time 2.0  --strafe-time 1.2
```

`eyes-on-me touchpad-test`: prints decoded signals and actions as you tap
and swipe, sends nothing to Spot. Also `--fake`: synthesises media-key
events with `CGEventPost` so the tap plumbing can be tested with no
headphones.

## 5. Safety rules (non-negotiable in review)

1. No single touch moves the robot for more than one bounded burst.
2. Every motion has `end_time_secs`; nothing continues if the process dies.
3. `stop` is the fastest path: first tap while moving is acted on the same
   tick, before any sequence detection.
4. `sit_stand` and `estop` require sequences; `estop` never has a one-touch
   binding in any profile.
5. If the event tap or volume listener fails to start, the run continues
   without touchpad and says so loudly; it never silently drops to a state
   where "stop" doesn't work.
6. Terminal `e`/`E` and window Space/Esc e-stops keep working regardless.

## 6. Known caveats

- **Multipoint**: with a phone also connected, the XM5 routes AVRCP to the
  device that last played audio. Disconnect the phone or play silent audio
  on the Mac (`afplay` a silent file on loop; some firmware needs audio for
  the head-tracker sensor anyway).
- **Volume hijack**: while `--touchpad` is on, swipes no longer change
  what you hear. Restored on exit.
- **Media keys from elsewhere**: a keyboard F7/F8/F9 or another BT device
  produces the same events. Acceptable for v1; v2 could filter by the HID
  sender if `CGEventGetIntegerValueField(kCGEventSourceUnixProcessID)` /
  sender ID proves usable.
- **Siri hold**: disable "Listen for Siri from headphones" or accept that a
  long press pops Siri.
- macOS only (Quartz/CoreAudio). Windows would use `WM_APPCOMMAND` /
  RawInput; out of scope.

## 7. Steps

| # | task                                                          | verify                                              |
| - | ------------------------------------------------------------- | --------------------------------------------------- |
| 1 | `gestures.py` + tests                                         | `pytest`                                            |
| 2 | `events.py`; `touchpad-test --fake` posts synthetic play/next/prev | decoded signals print; Music does not react     |
| 3 | `volume.py`; test with the XM5 on: swipe up/down ×5           | 5 signals, volume stays at baseline, restored on exit |
| 4 | `actions.py` + run-loop hook; dry-run prints actions          | `--dry-run --touchpad` logs `walk_fwd` etc.         |
| 5 | Hardware, pose mode: tap→recenter, ×3→sit/stand, ×5→estop     | add rows to `docs/testing.md` tier 2                |
| 6 | Hardware, turn mode: swipe up walks where you look            | bounded 0.8 m, stops on tap                         |
| 7 | `camera` profile; docs/run.md section                         |                                                     |

Deps to add: `pyobjc-framework-Quartz`, `pyobjc-framework-CoreAudio`
(darwin-only markers in `pyproject.toml`).

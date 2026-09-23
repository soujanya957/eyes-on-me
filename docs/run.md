# Run

Two terminals: one for the head tracker, one for Spot. Do [setup](setup.md) first.

**Terminal 1 — head tracker** (headphones paired and connected to the Mac):

```bash
./scripts/run-tracker.sh
```

You should see orientation lines scrolling. If it says
`macOS denied IOHID listen access`, grant **Input Monitoring** to the binary
(System Settings → Privacy & Security → Input Monitoring), then rerun.

**Terminal 2 — headphones only, see that tracking works:**

```bash
uv run eyes-on-me viz
```

Opens a window with a 3D head gizmo that follows yours, the raw tracker
angles, the mapped Spot angles, and bars showing how much of the body
envelope you're using (they turn red when clamped). `r` recenters, `a`
toggles the body/arm envelope, `q` quits. Turn left: the nose should swing to
the green (+y) side; look up: nose rises. If an axis is backwards, pass the
matching `--invert-*` flag (viz accepts the same mapping flags as `run`).

`uv run eyes-on-me --dry-run` prints the same numbers as text, no window.

**Terminal 2 — pre-flight the robot:**

```bash
uv run eyes-on-me check
```

Read-only. Checks, in order: network reachability, credentials, robot
ID/version, time sync, e-stop state and endpoints, who holds the body lease,
battery, motor state, system faults, arm present, camera sources, and whether
the head tracker is streaming locally. Fix anything marked `FAIL` before
running; `warn` lines are informational (e.g. the tablet holding the lease).

**Terminal 2 — drive Spot.** Each robot has its own git-ignored
`.env.<name>` holding `SPOT_IP`, `SPOT_USER`, `SPOT_PASS` (`cp .env.example
.env.tusker`). `--robot <name>` picks one; without it, the `SPOT_ROBOT` line in
`.env` decides (currently `tusker`). The log's first line says which robot and
IP it picked. Make
sure the Mac is on the robot's network (`ping <its SPOT_IP>`), the robot is on with
motors unlocked, and nothing else holds the lease (release control on the
tablet or it will be stolen — that's expected).

```bash
uv run eyes-on-me                 # pose mode: body tilt only, feet stay put
uv run eyes-on-me --robot rooter  # same, on rooter
```

What happens: connect → register e-stop → take lease → power on → stand →
**stands still, paused**. It does not follow your head until you press `s`.
Face forward, press `r` to make that straight-ahead, then `s` to start. Press
`s` again any time to stop: Spot squares up and holds still, and you can
recenter and restart from there. `--start-active` skips the pause and follows
immediately (the old behaviour).
**Ctrl+C** sits Spot and powers off (`--no-sit` to leave it standing).

Add `--viz` to get the head gizmo next to the robot:

```bash
uv run eyes-on-me --viz            # gizmo + robot, one process
```

This is a flag rather than a second terminal on purpose: `eyes-on-me viz`
binds the tracker's UDP port itself, so it **cannot** run at the same time as
`run` — whichever starts second dies with `Address already in use`. `--viz`
draws the same window from the control loop's own receiver.

Keys while running (terminal keys need Enter; window keys don't):

| action                          | terminal      | camera window | viz window |
| ------------------------------- | ------------- | ------------- | ---------- |
| **start/stop following**        | `s` + Enter   | —             | `s`        |
| **stand / walk** (pose ↔ turn)  | `w` + Enter   | —             | `w`        |
| recenter                        | `r` + Enter   | `r`           | `r`        |
| **E-STOP**: sit, then cut power | `e` + Enter   | `Space`       | `Space`    |
| **E-STOP**: cut power now       | `E` + Enter   | `Esc`         | `Esc`      |
| quit normally (sit, power off)  | Ctrl+C        | `q`           | `q`        |

`w` switches between standing (`pose`: the body leans where you look, feet
stay put) and walking (`turn`: Spot also steps round to face you). The banner
shows which one, e.g. **FOLLOWING WALK**. Straight-ahead carries over: after
walking round, standing again treats the direction Spot now faces as forward,
so you don't need to recenter. `w` does nothing in `--mode arm`.

While paused the log prefixes every line with `[paused]` and the viz window
shows a red **PAUSED** banner (green **FOLLOWING** once started). The mapped
body angles keep updating while paused, so you can see exactly what Spot
*would* do before you commit to it.

The software e-stop goes through the e-stop endpoint this process registers.
"Cut now" drops motor power immediately (Spot falls if standing); "sit, then
cut" is the SDK's `settle_then_cut`. After an e-stop the tool exits and
leaves motors cut; clear it from the tablet or just rerun. With
`--external-estop` the software e-stop is unavailable — use the tablet.

Once pose mode feels right, in open space:

```bash
uv run eyes-on-me --mode turn     # also steps round when you look past ±25°
```

A hostname on the command line overrides the robot's `SPOT_IP`. Common variants:

```bash
uv run eyes-on-me --no-invert-yaw              # yaw went the wrong way
uv run eyes-on-me --smoothing 0.2 --deadband 3 # calmer
uv run eyes-on-me --external-estop             # keep the tablet as e-stop
uv run eyes-on-me --body-height -0.1           # stand lower
```

## Touchpad (WH-1000XM5 earcup)

```bash
uv run eyes-on-me --viz --touchpad
```

| gesture | action |
| ------- | ------ |
| swipe forward | one step forward (0.5 m) |
| swipe back | one step back (0.4 m) |
| swipe up | one step **left** (0.4 m) |
| swipe down | one step **right** (0.4 m) |
| double tap | gripper open/close - or **abort the step** if one is running |
| double tap x5 in 2 s | E-STOP (settle, then cut) |

Steering stays with your head: a step goes where the body is pointing, so
look where you want to go, then swipe. Each step is a trajectory command with
a fixed relative goal, so a stray swipe moves Spot one step and stops - there
is no velocity left running.

A double tap means two things on purpose. While a step is running it aborts
immediately; idle, it toggles the gripper. Stopping must be the fastest thing
on the controller, and the touchpad has only five signals. The cost is that an
*idle* tap waits ~0.4 s before the gripper moves, in case it turns out to be
the first of the five that mean e-stop; an aborting tap is never delayed.

Single tap does nothing - the XM5 beeps but sends no command. Press-and-hold
is Siri and cover-the-cup is quick attention; neither reaches the Mac.

**Disconnect your phone first.** With multipoint the headset sends gestures
to whichever device last played audio, so with a phone connected the Mac sees
nothing at all - while head tracking keeps working, because that rides a
different link. Try the gestures with no robot first:

```bash
uv run eyes-on-me touchpad-test           # prints actions, moves nothing
uv run eyes-on-me touchpad-test --moving  # pretend a step runs: taps abort
```

While `--touchpad` is on, swipes no longer change what you hear; your volume
is restored on exit.

## Modes

| mode   | what happens                                                                                  |
| ------ | --------------------------------------------------------------------------------------------- |
| `pose` | Head yaw/pitch/roll → `synchro_stand_command` body orientation, clamped to ±25°/±20°/±12°.     |
| `turn` | Same, but yaw beyond the body envelope becomes a capped turn-in-place velocity. Heading is   |
|        | closed-loop on odometry, so Spot's *heading + body yaw* converges to your head yaw.           |
| `arm`  | Spot Arm only. Stands, unstows, lifts the gripper to a raised ready pose in front of the body, |
|        | your head drives the gripper's orientation (±70°/±50°/±30°) via `arm_pose_command`. The      |
|        | body stays still. The hand camera is shown by default. Arm is stowed on exit.               |

## Cameras

`--camera` opens a window with a live Spot camera:

```bash
uv run eyes-on-me --camera auto              # body camera that faces where you're looking
uv run eyes-on-me --camera back              # fixed source
uv run eyes-on-me --mode arm                 # hand camera by default
uv run eyes-on-me --mode arm --camera none   # arm without a window
```

Sources: `front`, `front-right`, `left`, `right`, `back`, `hand`. In the window:
`1`–`6` pick a camera, `0` back to auto, `r` recenter, `q` quit. Frames are
fetched at ~10 fps on a background thread; the control loop is unaffected.

## Arm mode

```bash
uv run eyes-on-me --mode arm                       # hand hovers at (0.6, 0, 0.55) m
uv run eyes-on-me --mode arm --hand-pos 0.7 0 0.6  # higher / further out
uv run eyes-on-me --mode arm --arm-max-yaw 90      # wider sweep
```

Position is in Spot's `flat_body` frame (x forward, y left, z up). Keep the
hand clear of the body and the ground: at the default pose the arm has room
to pitch down ~50° without touching anything, but check your `--hand-pos`
with the arm's reach before widening the pitch limit. Ctrl+C stows the arm
before sitting.

## Tuning flags

`--max-yaw/--max-pitch/--max-roll`, `--arm-max-*`, `--*-gain`, `--deadband`, `--smoothing`,
`--no-invert-yaw`, `--no-invert-pitch`, `--invert-roll`, `--turn-kp`,
`--max-turn-rate`, `--rate`. Run `uv run eyes-on-me -h` for defaults.

Tracker pitch is inverted by default (tracker +pitch = look up, Spot +pitch =
nose down). If Spot moves the wrong way on any axis, flip it with the matching
`--invert-*` flag; `--dry-run` shows the signed body targets before anything moves.

## Safety

* By default the tool registers its own e-stop endpoint (like the SDK's `wasd`
  example) and takes the body lease; `e`/`E` in the terminal or Space/Esc in
  the camera window trigger it. If this process dies, the endpoint stops
  checking in and Spot stops within the 9 s timeout. Pass `--external-estop`
  to keep the tablet/estop GUI in charge instead.
* `turn` mode makes Spot rotate; `arm` mode swings the arm through a wide
  envelope. Start in `pose` mode, in open space, with the e-stop within reach.
* The UDP stream is unauthenticated loopback; don't forward it off-host.

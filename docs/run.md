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

**Terminal 2 — drive Spot.** Credentials come from a git-ignored `.env`
(`cp .env.example .env`, then set `SPOT_IP`, `SPOT_USER`, `SPOT_PASS`). Make
sure the Mac is on the robot's network (`ping $SPOT_IP`), the robot is on with
motors unlocked, and nothing else holds the lease (release control on the
tablet or it will be stolen — that's expected).

```bash
uv run eyes-on-me                 # pose mode: body tilt only, feet stay put
```

What happens: connect → register e-stop → take lease → power on → stand →
first head sample becomes "straight ahead" → body follows your head at 20 Hz.
**Ctrl+C** sits Spot and powers off (`--no-sit` to leave it standing).

Keys while running (terminal keys need Enter; camera-window keys don't):

| action                          | terminal      | camera window |
| ------------------------------- | ------------- | ------------- |
| recenter                        | `r` + Enter   | `r`           |
| **E-STOP**: sit, then cut power | `e` + Enter   | `Space`       |
| **E-STOP**: cut power now       | `E` + Enter   | `Esc`         |
| quit normally (sit, power off)  | Ctrl+C        | `q`           |

The software e-stop goes through the e-stop endpoint this process registers.
"Cut now" drops motor power immediately (Spot falls if standing); "sit, then
cut" is the SDK's `settle_then_cut`. After an e-stop the tool exits and
leaves motors cut; clear it from the tablet or just rerun. With
`--external-estop` the software e-stop is unavailable — use the tablet.

Once pose mode feels right, in open space:

```bash
uv run eyes-on-me --mode turn     # also steps round when you look past ±25°
```

A hostname on the command line overrides `SPOT_IP`. Common variants:

```bash
uv run eyes-on-me --invert-yaw                 # yaw went the wrong way
uv run eyes-on-me --smoothing 0.2 --deadband 3 # calmer
uv run eyes-on-me --external-estop             # keep the tablet as e-stop
uv run eyes-on-me --body-height -0.1           # stand lower
```

## Modes

| mode   | what happens                                                                                  |
| ------ | --------------------------------------------------------------------------------------------- |
| `pose` | Head yaw/pitch/roll → `synchro_stand_command` body orientation, clamped to ±25°/±20°/±12°.     |
| `turn` | Same, but yaw beyond the body envelope becomes a capped turn-in-place velocity. Heading is   |
|        | closed-loop on odometry, so Spot's *heading + body yaw* converges to your head yaw.           |
| `arm`  | Spot Arm only. Stands, unstows, lifts the gripper to a look pose in front of the body, then   |
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
uv run eyes-on-me --mode arm                       # hand hovers at (0.6, 0, 0.45) m
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
`--invert-yaw`, `--no-invert-pitch`, `--invert-roll`, `--turn-kp`,
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

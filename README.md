# eyes-on-me

Active perception for Boston Dynamics Spot driven by your head: look left and
Spot looks left. Head orientation comes from the IMU inside Sony WH-1000XM5
headphones (via [sony-head-tracker](https://github.com/soujanya957/sony-head-tracker),
included as a submodule); Spot moves through the official `bosdyn-client` SDK.

```
WH-1000XM5 ──BT──▶ sony-head-tracker ──UDP JSON :4243──▶ eyes-on-me ──gRPC──▶ Spot
                   (macOS bridge)                        (this package)
```

## Setup (macOS 14+, Apple Silicon or Intel)

```bash
git clone --recurse-submodules https://github.com/soujanya957/eyes-on-me.git
cd eyes-on-me
uv sync            # Python 3.12 venv + bosdyn-client
```

Build the head-tracker bridge (needs Xcode + CMake ≥ 3.25):

```bash
./scripts/run-tracker.sh probe    # read-only check that the XM5 is visible
./scripts/run-tracker.sh          # starts the bridge; leave it running
```

First run: grant the binary **Input Monitoring** in System Settings, then
rerun. See `sony-head-tracker/docs/MACOS.md` for pairing and troubleshooting.

## Run

Two terminals: one for the head tracker, one for Spot.

**Terminal 1 — head tracker** (headphones paired and connected to the Mac):

```bash
./scripts/run-tracker.sh
```

You should see orientation lines scrolling. If it says
`macOS denied IOHID listen access`, grant **Input Monitoring** to the binary
(System Settings → Privacy & Security → Input Monitoring), then rerun.

**Terminal 2 — sanity check the mapping, no robot:**

```bash
uv run eyes-on-me --dry-run
```

Turn your head; `body y/p/r` should follow. Left should be positive yaw,
looking up should be negative pitch (Spot's nose-up). If an axis is backwards,
note the matching `--invert-*` flag for later.

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
Press `r` + Enter any time to recenter. **Ctrl+C** sits Spot and powers off
(`--no-sit` to leave it standing).

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

### Modes

| mode   | what happens                                                                                  |
| ------ | --------------------------------------------------------------------------------------------- |
| `pose` | Head yaw/pitch/roll → `synchro_stand_command` body orientation, clamped to ±25°/±20°/±12°.     |
| `turn` | Same, but yaw beyond the body envelope becomes a capped turn-in-place velocity. Heading is   |
|        | closed-loop on odometry, so Spot's *heading + body yaw* converges to your head yaw.           |

### Tuning flags

`--max-yaw/--max-pitch/--max-roll`, `--*-gain`, `--deadband`, `--smoothing`,
`--invert-yaw`, `--no-invert-pitch`, `--invert-roll`, `--turn-kp`,
`--max-turn-rate`, `--rate`. Run `uv run eyes-on-me -h` for defaults.

Tracker pitch is inverted by default (tracker +pitch = look up, Spot +pitch =
nose down). If Spot moves the wrong way on any axis, flip it with the matching
`--invert-*` flag; `--dry-run` shows the signed body targets before anything moves.

## Safety

* By default the tool registers its own e-stop endpoint (like the SDK's `wasd`
  example) and takes the body lease. Pass `--external-estop` to keep the
  tablet/estop GUI in charge instead.
* `turn` mode makes Spot rotate. Start in `pose` mode, in open space, with the
  e-stop within reach.
* The UDP stream is unauthenticated loopback; don't forward it off-host.

## Layout

```
eyes_on_me/
  head_tracker.py   UDP JSON receiver (latest-sample, drops stale)
  gaze_mapper.py    pure math: recenter, deadband, gain, smoothing, clamp, turn split
  spot_gaze.py      lease / e-stop / power / control loop
  cli.py            argparse entry point (`eyes-on-me`)
tests/              pytest, no robot needed
sony-head-tracker/  submodule (fork of NicholasSlattery/sony-head-tracker)
scripts/run-tracker.sh
```

```bash
uv run pytest
```

## Ideas: what else head tracking + Spot can do

Everything below builds on the same `HeadSample` → `GazeMapper` → command
pipeline; most are a new mode in `spot_gaze.py` plus a mapper function.

**Perception**
- **Gaze-directed camera** — pick which of Spot's five body cameras (or the
  Spot CAM PTZ) to stream based on head yaw, so you get a live "what Spot sees
  where I'm looking" view. With the PTZ, drive pan/tilt 1:1 with your head.
- **Look-then-walk** — in `turn` mode, hold your gaze for ~1.5 s and Spot
  walks a step toward what you're looking at (`synchro_trajectory_command_in_body_frame`).
- **Attention logging** — record where you looked and what Spot's camera saw
  there; a cheap way to collect "human-salient" frames for a dataset.

**Teleop / control**
- **Head as joystick** — pitch forward/back → v_x, roll → v_y, yaw → v_rot.
  Hands-free driving; a WASD-free `wasd.py`.
- **Nod / shake gestures** — a quick pitch oscillation = "yes" (confirm, sit),
  yaw oscillation = "no" (cancel). The gyroscope field in the JSON stream is
  perfect for this — threshold on angular velocity, not angle.
- **Dual mode with the arm** — if your Spot has an arm, map head to
  gripper-camera pose (`arm_pose_command`) so you inspect things by looking.

**Interaction**
- **Mutual gaze** — combine with a face detector on Spot's front camera: when
  Spot sees you, it turns to face you; you turn your head, it mirrors. Two
  agents doing active perception at each other.
- **Spatial audio loop** — the XM5 already does head-tracked audio. Feed Spot's
  microphone (or a beamformed direction) back so sounds from Spot's side stay
  spatially fixed as you turn — a telepresence "ears on the robot".
- **Follow-me with anticipation** — normal follow uses your body position; add
  head yaw as a lead term so Spot starts turning before you do.

**Research angles**
- Compare fixed vs. head-driven camera selection for object-search tasks.
- Use head yaw as a prior in a visual SLAM / semantic mapping loop (attention
  weighting for keyframe selection).
- Latency study: BT HID (~40 ms) + gRPC + Spot's body controller — measure the
  end-to-end lag and whether smoothing or prediction (extrapolating with the
  gyroscope) helps.

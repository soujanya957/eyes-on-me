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

Check the mapping without a robot (prints what Spot *would* do):

```bash
uv run eyes-on-me --dry-run
```

Put your robot's address and credentials in a git-ignored `.env`:

```bash
cp .env.example .env   # then edit SPOT_IP / SPOT_USER / SPOT_PASS
```

Then drive Spot (a hostname on the command line overrides `SPOT_IP`; if no
password is set you'll be prompted):

```bash
uv run eyes-on-me                 # body tilt only, robot stays put
uv run eyes-on-me --mode turn     # also steps round for large yaw
```

Face forward and press `r` + Enter to recenter. Ctrl+C sits Spot and powers
off (`--no-sit` to leave it standing).

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

# Testing

Two tiers. Tier 1 needs nothing but this laptop and runs in under a minute.
Tier 2 is a checklist to walk through with the headphones and the robot
before you trust the system on real hardware.

## Does it actually work with the WH-1000XM5?

**Yes — confirmed on this Mac, 2026-09-22.** `probe` reports
`description=#AndroidHeadTracker#1.0`, `verified Android tracker=yes`, and the
bridge streams YPR at ~25 pps. Details: WH-1000XM5 firmware 2.5.1,
macOS 26.5 (build 25F71, Darwin 25.5.0), arm64.

Upstream lists the XM5 as "protocol-compatible; macOS hardware confirmation
requested" — the only macOS hardware-validated model in their table is the
ULT WEAR. They are asking for exactly this report (model, firmware, macOS
version, architecture, redacted `probe` output), so it is worth sending.

Note the gyroscope fields read all zeros on this firmware. That is harmless
here: the pipeline uses only the rotation vector, and `fake-tracker.py`
already sends `gyroscope: None`.

---

## Tier 1 — software only (run now)

```bash
./scripts/test-software.sh
```

What it covers, and how to run each piece by hand:

| # | test | command | pass criteria |
| - | ---- | ------- | ------------- |
| 1 | unit tests: angle wrap, deadband, clamp, recenter, smoothing, turn split, arm envelope, JSON parse, yaw→camera | `uv run pytest -q` | `20 passed` |
| 2 | CLI parses | `uv run eyes-on-me -h`, `viz -h`, `check -h` | usage text, exit 0 |
| 3 | Spot SDK present, every API we call exists | (inside the script) | no `AssertionError` |
| 4 | end-to-end dry run, all three modes, against a synthetic tracker | see below | `body y= … v_rot=` lines, no traceback |
| 5 | `check` fails fast on an unreachable host | `uv run eyes-on-me check 127.0.0.1` | `[FAIL] network … unreachable` within ~3 s |
| 6 | tracker bridge compiles | `cmake` (skipped without Xcode) | `Built target sony-head-tracker-macos` |

### Synthetic tracker

`scripts/fake-tracker.py` sends the same JSON the real bridge sends, so
anything that reads port 4243 can be exercised without headphones:

```bash
uv run scripts/fake-tracker.py             # terminal 1: yaw sweeps ±60°
uv run eyes-on-me viz                      # terminal 2: gizmo should sweep left/right
uv run eyes-on-me --dry-run --mode turn    # or: numbers only
uv run scripts/fake-tracker.py --step 40   # jumps 0 ↔ 40° every 2 s: watch smoothing + clamp
```

Things worth eyeballing in `viz` with the fake tracker:

- yaw bar goes red at ±25 while raw yaw keeps climbing to 60 (clamp works)
- with `--step 40`, the gizmo eases into the new pose over a few frames
  rather than snapping (smoothing works); `--smoothing 1` makes it snap
- `a` toggles the arm envelope: the same 40° is now inside ±70 and the bar is green
- `r` recenters: whatever pose the fake tracker is at becomes zero

---

## Tier 2 — hardware, in order

Do these top to bottom. Each step has a stop condition; don't move on past a
failed one.

### H. Headphones only

| # | step | pass | if it fails |
| - | ---- | ---- | ----------- |
| H1 | Pair the XM5 to the Mac (Bluetooth settings), then `./scripts/run-tracker.sh probe` | prints the `#AndroidHeadTracker#` descriptor, exit 0 | exit 2 + "denied IOHID": grant Input Monitoring and rerun. **Exit 2 with no sensor listed: turn the Mac's Bluetooth off and on and restart the headphones** — see below. Then: XM5 *connected*, not just paired; firmware current in Sony Sound Connect; no phone holding the sensor (multipoint); `sony-head-tracker/docs/MACOS.md` |
| H2 | `./scripts/run-tracker.sh` (bridge) | orientation lines scroll at ~25 pkt/s | 0 pkt/s: some XM5 firmware only streams the sensor while audio is playing — start any audio |
| H3 | `uv run eyes-on-me viz` | gizmo follows your head with < ~100 ms lag; pkt/s ≈ 25 | "waiting for tracker": bridge not running or wrong port |
| H4 | Axis check in `viz`: turn head **left** | nose swings toward the green (+y) arrow; `spot yaw` positive | negative: use `--no-invert-yaw` from now on |
| H5 | Look **up** | nose rises; `spot pitch` **negative** (Spot nose-up) | positive: use `--no-invert-pitch` |
| H6 | Tilt head **right** (right ear down) | `spot roll` positive | negative: use `--invert-roll` |
| H7 | Sit still 60 s | angles drift < 2° | more: press `r`; if it keeps drifting, raise `--deadband` |
| H8 | Turn 90° left, return | returns to ~0 (no yaw slip) | slips: note it; the headset re-zeros itself sometimes (`resets` counter) |

Write down the invert flags you needed; you'll pass them to `run` every time.

**If `probe` finds nothing, check the service list first** — it tells you
whether the headset ever offered the sensor:

```bash
system_profiler SPBluetoothDataType | grep -A8 WH-1000XM5 | grep Services
```

`< HFP AVRCP A2DP ACL >` means **no HID** — the tracker interface was never
published, so no amount of probing will find it. `< HFP AVRCP A2DP HID ACL >`
is what you want. Toggling the Mac's Bluetooth off and on and restarting the
headphones rebuilds the service record and is what fixed it here; macOS
otherwise caches a stale record from pairing time. Playing audio does *not*
affect this (it matters for H2, not H1).

### TP. Touchpad, no robot

Phone's Bluetooth **off** (multipoint sends gestures to the phone instead).

| # | step | pass | if it fails |
| - | ---- | ---- | ----------- |
| TP1 | `uv run eyes-on-me touchpad-test` | both channels report ready | "no volume channel": device exposes no readable volume. "no transport commands": MediaPlayer missing |
| TP2 | swipe forward / back | `^ forward` / `v back` | nothing at all: disconnect the phone |
| TP3 | swipe up / down | `< left` / `> right` | only these work: the MediaRemote side is not registered |
| TP4 | double tap, wait a second | `[] gripper open/close` after ~0.4 s | fires instantly: `--tap-settle` is 0 |
| TP5 | `touchpad-test --moving`, double tap | `! STOP` immediately, no gripper | delayed: aborts must never wait |
| TP6 | double tap 5x quickly | one `!! E-STOP`, no gripper toggles | several gripper lines: taps fell outside the 2 s window |

### S. Spot, no motion

| # | step | pass | if it fails |
| - | ---- | ---- | ----------- |
| S1 | Mac on the robot's Wi-Fi; `.env.<robot>` filled in | `ping <SPOT_IP>` replies | wrong network / IP |
| S2 | `uv run eyes-on-me check` | every line `ok` or `warn`; battery > 30 %; no faults; head tracker line `ok` | `FAIL` auth: credentials. `FAIL` time sync: retry once |
| S3 | Release control on the tablet (or leave it; `run` takes the lease and the tablet will say so) | — | — |

### P. Pose mode (body only, feet stay planted)

Open space, robot on flat ground, e-stop (tablet or the terminal) within reach. One person watches the robot, one runs the terminal.

| # | step | pass | if it fails |
| - | ---- | ---- | ----------- |
| P1 | `uv run eyes-on-me --viz` (+ your invert flags) | log shows: powering on → standing → `PAUSED`; Spot stands and holds still; viz window shows a red **PAUSED** banner | Spot moves before you press `s`: stop and report it, pausing is supposed to freeze the body |
| P1b | Face forward, `r` + Enter, then `s` + Enter | log `recentred ...` then `FOLLOWING your head.`; banner turns green | recenter while paused should always work; if `s` does nothing, check the terminal has focus |
| P2 | Slowly turn head left 20° | Spot's body yaws left, stops at ~20° | wrong direction: press `s` to pause, Ctrl+C, add/remove `--no-invert-yaw` |
| P3 | Look up / down | body pitches nose-up / nose-down | wrong: toggle `--no-invert-pitch` |
| P4 | Tilt head | body rolls the same way, less (roll gain 0.5) | wrong: `--invert-roll` |
| P5 | Turn head 90° | body stops at 25°, doesn't strain | — |
| P6 | Fast head shake | body follows smoothly, no oscillation | jitter: `--smoothing 0.2`; lag: `--smoothing 0.6` |
| P6b | Press `s` mid-motion with your head turned | Spot squares up to level and freezes there; log `[paused]` | body keeps following: report it |
| P7 | **E-stop drill**: type `e` + Enter | log `E-STOP: settling`, Spot sits, motors cut, tool exits | nothing happens: you ran with `--external-estop`; use the tablet |
| P8 | Rerun; Ctrl+C | Spot sits, powers off cleanly | — |

### T. Turn mode (Spot rotates in place)

| # | step | pass | if it fails |
| - | ---- | ---- | ----------- |
| T1 | `uv run eyes-on-me --mode turn` | as P1 | — |
| T2 | Turn head 60° left and hold | body yaws to 25°, then Spot steps round; body yaw shrinks as it turns; stops when heading + body yaw ≈ 60° | overshoots/oscillates: `--turn-kp 0.8`; too slow: `--turn-kp 2` or `--max-turn-rate 1.2` |
| T3 | Look back to centre | Spot steps back to its original heading, body straight | drifts: odometry slip; press `r` to make the current heading the new zero |
| T4 | Turn 180° | Spot turns fully round (takes a few seconds, rate-capped) | — |
| T5 | E-stop drill with **Esc** in a `--camera auto` window | immediate cut | — |

### A. Arm mode (Spot Arm only)

Clear 1 m in front of and above the robot. Gripper is empty.

| # | step | pass | if it fails |
| - | ---- | ---- | ----------- |
| A1 | `uv run eyes-on-me --mode arm` | stand → "unstowing arm" → "raising hand to look pose" → hand hovers 0.6 m ahead, 0.45 m up, pointing forward; hand-cam window opens | arm doesn't move: `check` said no arm |
| A2 | Turn head left/right | gripper yaws same way, body still; hand cam pans | — |
| A3 | Look up/down | gripper pitches; does **not** approach the body or ground at ±50° | close call: lower `--arm-max-pitch 30` or raise `--hand-pos 0.6 0 0.55` |
| A4 | Tilt head | gripper rolls | — |
| A5 | Fast head motion | arm follows without jerking | jerky: `--smoothing 0.2` |
| A6 | Ctrl+C | "stowing arm" → sits → power off | — |
| A7 | E-stop drill (Space in the window) while arm is raised | settle-then-cut: arm and body settle, then cut | — |

### After

Record in this file (or an issue): Mac/macOS version, XM5 firmware, Spot
software version (from `check`), the invert flags you needed, and any step
that failed. If H1 passes, consider reporting XM5-on-macOS to the upstream
sony-head-tracker project — they're asking for exactly that confirmation.

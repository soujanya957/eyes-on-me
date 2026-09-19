#!/usr/bin/env bash
# Everything that can be verified with no headphones and no robot. Exit 0 = all green.
set -uo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
fail=0
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }
check() { if "$@"; then echo "   ok"; else echo "   FAIL"; fail=1; fi; }

step "1. unit tests (mapper, parser, camera selection)"
check uv run pytest -q

step "2. CLI entry points parse"
check bash -c "uv run eyes-on-me -h >/dev/null"
check bash -c "uv run eyes-on-me viz -h >/dev/null"
check bash -c "uv run eyes-on-me check -h >/dev/null"

step "3. Spot SDK imports and API surface"
check uv run python -c "
from bosdyn.client.robot_command import RobotCommandBuilder as B, block_until_arm_arrives, blocking_stand
from bosdyn.client.estop import EstopKeepAlive
from bosdyn.client.image import ImageClient
from bosdyn.geometry import EulerZXY
import eyes_on_me.spot_gaze, eyes_on_me.camera, eyes_on_me.viz, eyes_on_me.check
for f in ('synchro_stand_command','synchro_velocity_command','arm_pose_command','arm_ready_command','arm_stow_command','mobility_params'):
    assert hasattr(B, f), f
for f in ('stop','settle_then_cut'): assert hasattr(EstopKeepAlive, f), f
"

step "4. end-to-end dry run against a fake tracker (pose, turn, arm)"
uv run scripts/fake-tracker.py >/dev/null 2>&1 &
FAKE=$!
sleep 0.5
for mode in pose turn arm; do
  out=$(uv run python -c "
import threading, os, signal
threading.Timer(1.5, lambda: os.kill(os.getpid(), signal.SIGINT)).start()
from eyes_on_me.cli import main; main(['--dry-run','--mode','$mode','--no-recenter'])
" </dev/null 2>&1)
  if grep -q "body y=" <<<"$out" && ! grep -q "Traceback" <<<"$out"; then echo "   $mode ok"; else echo "   $mode FAIL"; echo "$out" | tail -5; fail=1; fi
done
kill $FAKE 2>/dev/null

step "5. check command fails fast on an unreachable host"
out=$(uv run eyes-on-me check 127.0.0.1 2>&1)
if grep -q "unreachable" <<<"$out"; then echo "   ok"; else echo "   FAIL: $out"; fail=1; fi

step "6. head-tracker bridge builds (needs Xcode + CMake; skipped if absent)"
if command -v cmake >/dev/null && [[ -d /Applications/Xcode.app ]]; then
  check cmake -S sony-head-tracker -B sony-head-tracker/build/macos -DCMAKE_BUILD_TYPE=Release
  check cmake --build sony-head-tracker/build/macos --target sony-head-tracker-macos --parallel
else
  echo "   skipped"
fi

echo
if [[ $fail -eq 0 ]]; then echo "ALL SOFTWARE TESTS PASSED"; else echo "SOME TESTS FAILED"; fi
exit $fail

#!/usr/bin/env bash
# Build (once) and run the sony-head-tracker CLI bridge from the submodule.
# Emits OpenTrack doubles on 127.0.0.1:4242 and JSON on :4243.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/sony-head-tracker"
BIN="$ROOT/build/macos/sony-head-tracker-macos"
if [[ ! -x "$BIN" ]]; then
  cmake -S "$ROOT" -B "$ROOT/build/macos" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$ROOT/build/macos" --target sony-head-tracker-macos --parallel
fi
exec "$BIN" "${1:-bridge}"

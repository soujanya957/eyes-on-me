# Setup

macOS 14+, Apple Silicon or Intel.

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
rerun. See `../sony-head-tracker/docs/MACOS.md` for pairing and troubleshooting.

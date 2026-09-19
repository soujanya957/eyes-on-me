"""``eyes-on-me check``: is Spot reachable, and is it in a state we can drive?"""

from __future__ import annotations

import socket
import sys
import time

from bosdyn.api import estop_pb2, robot_state_pb2
from bosdyn.client import create_standard_sdk
from bosdyn.client.estop import EstopClient
from bosdyn.client.exceptions import RpcError
from bosdyn.client.image import ImageClient
from bosdyn.client.lease import LeaseClient
from bosdyn.client.robot_id import RobotIdClient
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.client.util import authenticate

OK, WARN, BAD = "\033[32m ok \033[0m", "\033[33mwarn\033[0m", "\033[31mFAIL\033[0m"


def line(status: str, label: str, detail: str = "") -> None:
    print(f"[{status}] {label:<18} {detail}")


def main(hostname: str, tracker_port: int) -> int:
    failures = 0

    # 1. TCP reachability on the gRPC port before the SDK's long retry timeouts.
    t0 = time.monotonic()
    try:
        with socket.create_connection((hostname, 443), timeout=3):
            line(OK, "network", f"{hostname}:443 reachable in {1000 * (time.monotonic() - t0):.0f} ms")
    except OSError as e:
        line(BAD, "network", f"{hostname}:443 unreachable ({e}). Same Wi-Fi as the robot? Correct SPOT_IP?")
        return 1

    # 2. Auth.
    sdk = create_standard_sdk("eyes-on-me-check")
    robot = sdk.create_robot(hostname)
    try:
        authenticate(robot)
        line(OK, "auth", "credentials accepted")
    except Exception as e:
        line(BAD, "auth", f"{type(e).__name__}: {e}. Check SPOT_USER / SPOT_PASS in .env")
        return 1

    rid = robot.ensure_client(RobotIdClient.default_service_name).get_id()
    line(OK, "robot", f"{rid.nickname or '(no nickname)'}  serial {rid.serial_number}  sw {rid.software_release.version.major_version}."
                      f"{rid.software_release.version.minor_version}.{rid.software_release.version.patch_level}")

    try:
        robot.time_sync.wait_for_sync(timeout_sec=5)
        line(OK, "time sync", f"skew {robot.time_sync.get_robot_clock_skew().ToMilliseconds()} ms")
    except Exception as e:
        line(BAD, "time sync", str(e)); failures += 1

    # 3. E-stop.
    estop = robot.ensure_client(EstopClient.default_service_name).get_status()
    lvl = estop_pb2.EstopStopLevel.Name(estop.stop_level).replace("ESTOP_LEVEL_", "")
    eps = ", ".join(f"{ep.endpoint.name}({'ok' if ep.stop_level == estop_pb2.ESTOP_LEVEL_NONE else 'STOP'})" for ep in estop.endpoints) or "none"
    if estop.stop_level == estop_pb2.ESTOP_LEVEL_NONE:
        line(OK, "e-stop", f"clear; endpoints: {eps}")
    else:
        line(WARN, "e-stop", f"level {lvl}; endpoints: {eps}. `run` will register its own endpoint unless --external-estop")

    # 4. Lease.
    leases = robot.ensure_client(LeaseClient.default_service_name).list_leases()
    body = next((l for l in leases if l.resource == "body"), None)
    owner = body.lease_owner.client_name if body and body.lease_owner.client_name else ""
    if owner:
        line(WARN, "lease", f"body held by '{owner}' ({body.lease_owner.user_name}); `run` will take it")
    else:
        line(OK, "lease", "body lease free")

    # 5. Power / battery / state.
    st = robot.ensure_client(RobotStateClient.default_service_name).get_robot_state()
    ps = st.power_state
    motor = robot_state_pb2.PowerState.MotorPowerState.Name(ps.motor_power_state).replace("STATE_", "").lower()
    batt = st.battery_states[0] if st.battery_states else None
    if batt:
        pct = batt.charge_percentage.value
        temp = max(batt.temperatures) if batt.temperatures else float("nan")
        line(OK if pct > 20 else WARN, "battery", f"{pct:.0f}%  {batt.voltage.value:.1f} V  max cell {temp:.0f} °C")
    line(OK, "motors", motor)
    faults = [f.name for f in st.system_fault_state.faults]
    if faults:
        line(WARN, "faults", ", ".join(faults))
    else:
        line(OK, "faults", "none")

    # 6. Arm and cameras.
    line(OK, "arm", "present" if robot.has_arm() else "none (`--mode arm` unavailable)")
    try:
        srcs = {s.name for s in robot.ensure_client(ImageClient.default_service_name).list_image_sources()}
        want = ["frontleft_fisheye_image", "frontright_fisheye_image", "left_fisheye_image", "right_fisheye_image", "back_fisheye_image", "hand_color_image"]
        have = [w for w in want if w in srcs]
        line(OK, "cameras", f"{len(have)}/{len(want)} known sources; {len(srcs)} total")
    except RpcError as e:
        line(WARN, "cameras", str(e))

    # 7. Head tracker (local).
    from .head_tracker import HeadTrackerReceiver

    rx = HeadTrackerReceiver(port=tracker_port).start()
    time.sleep(0.6)
    s = rx.latest()
    rx.stop()
    if s:
        line(OK, "head tracker", f"127.0.0.1:{tracker_port} live  yaw {s.yaw:.1f} pitch {s.pitch:.1f} roll {s.roll:.1f}")
    else:
        line(WARN, "head tracker", f"nothing on 127.0.0.1:{tracker_port}; start ./scripts/run-tracker.sh")

    print("\nready to run" if failures == 0 else f"\n{failures} problem(s)", file=sys.stderr)
    return 1 if failures else 0

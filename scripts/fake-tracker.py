#!/usr/bin/env python3
"""Emit synthetic sony-head-tracker JSON datagrams so viz/dry-run work without headphones.

    uv run scripts/fake-tracker.py            # slow yaw sweep ±60°, gentle pitch
    uv run scripts/fake-tracker.py --still    # constant zero pose
    uv run scripts/fake-tracker.py --step 40  # jump between 0 and +40° yaw every 2 s
"""
import argparse
import json
import math
import socket
import time

p = argparse.ArgumentParser()
p.add_argument("--port", type=int, default=4243)
p.add_argument("--hz", type=float, default=25.0)
p.add_argument("--still", action="store_true")
p.add_argument("--step", type=float, metavar="DEG")
a = p.parse_args()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t0 = time.monotonic()
print(f"fake tracker -> 127.0.0.1:{a.port} at {a.hz:.0f} Hz (Ctrl+C to stop)")
try:
    while True:
        t = time.monotonic() - t0
        if a.still:
            yaw, pitch, roll = 0.0, 0.0, 0.0
        elif a.step is not None:
            yaw, pitch, roll = (a.step if int(t / 2) % 2 else 0.0), 0.0, 0.0
        else:
            yaw, pitch, roll = 60 * math.sin(t / 3), 15 * math.sin(t / 5), 5 * math.sin(t / 7)
        msg = {
            "version": 2, "device": "FAKE-XM5",
            "yprDegrees": [round(yaw, 2), round(pitch, 2), round(roll, 2)],
            "quaternion": [1, 0, 0, 0], "rotationVector": [0, 0, 0],
            "gyroscope": None, "accelerometer": None,
            "resetCounter": 0, "packetsPerSecond": a.hz, "receiveLatencyMs": -1.0,
        }
        sock.sendto(json.dumps(msg).encode(), ("127.0.0.1", a.port))
        time.sleep(1.0 / a.hz)
except KeyboardInterrupt:
    pass

"""Synthetic ZMQ publisher for smoke-testing the WebXR-bridge -> Isaac Sim path.

Stands in for telegrip when you want to verify the bridge works end-to-end
without the headset / IK in the loop. Publishes a slow sinusoid sweep on each
joint so the sim arms should clearly move when the bridge is wired up.

Usage (from the project root, with the leisaac sim already running):

    python scripts/zmq_sim_smoketest.py

The sim must have been launched with `--teleop_device bi-so101webxr` and the
default `--webxr_endpoint tcp://127.0.0.1:5555`.
"""

from __future__ import annotations

import json
import math
import time

import zmq

ENDPOINT = "tcp://*:5555"
RATE_HZ = 30
PERIOD_S = 8.0  # one full sweep

JOINT_AMPLITUDES_DEG = [
    20.0,   # shoulder_pan
    15.0,   # shoulder_lift
    25.0,   # elbow_flex
    10.0,   # wrist_flex
    30.0,   # wrist_roll
    20.0,   # gripper (sweeps within sim's 0..100 range)
]
LEFT_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
RIGHT_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


def main() -> None:
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 1)
    sock.bind(ENDPOINT)
    print(f"[smoke] bound on {ENDPOINT}, sweeping joints at {RATE_HZ} Hz")
    print("[smoke] left arm sweeps in phase, right arm sweeps phase-shifted")

    period = 1.0 / RATE_HZ
    t0 = time.monotonic()
    try:
        while True:
            t = time.monotonic() - t0
            phase = 2.0 * math.pi * (t / PERIOD_S)
            left = [
                LEFT_HOME[i] + JOINT_AMPLITUDES_DEG[i] * math.sin(phase)
                for i in range(6)
            ]
            right = [
                RIGHT_HOME[i] + JOINT_AMPLITUDES_DEG[i] * math.sin(phase + math.pi)
                for i in range(6)
            ]
            sock.send_string(json.dumps({"left": left, "right": right}))
            time.sleep(period)
    except KeyboardInterrupt:
        print("\n[smoke] stopping")
    finally:
        sock.close(linger=0)


if __name__ == "__main__":
    main()

"""Drive both SO-101 arms to the saved home pose.

Uses raw scservo SDK so we don't trigger lerobot's connect-time torque drop
or hit its flaky strict ping. Home pose is raw motor counts saved by
snap_home.py.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import time

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_POSE = os.path.join(HERE, "home_pose.json")
EXAMPLE_POSE = os.path.join(HERE, "home_pose.example.json")
PORTS = {
    "left":  os.environ.get("XRTACT_LEFT_PORT",  "/dev/ttyACM0"),
    "right": os.environ.get("XRTACT_RIGHT_PORT", "/dev/ttyACM1"),
}


def home_one_arm(side: str, port: str, target: dict[str, int],
                 duration: float, rate: float) -> None:
    print(f"\n[{side}] connecting to {port}")
    ph = PortHandler(port)
    if not (ph.openPort() and ph.setBaudRate(1_000_000)):
        print(f"[{side}] could not open {port}"); return
    pkt = PacketHandler(0)

    start = {}
    for sid_str, _ in target.items():
        sid = int(sid_str)
        for _ in range(10):
            pos, comm, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
            if comm == COMM_SUCCESS:
                start[sid_str] = int(pos); break
        else:
            print(f"[{side}] id={sid}: could not read present_position"); ph.closePort(); return

    print(f"[{side}] start -> target:")
    for sid_str in sorted(target.keys(), key=int):
        print(f"    id={sid_str}  {start[sid_str]:>5d} -> {target[sid_str]:>5d}  "
              f"(delta {target[sid_str] - start[sid_str]:+5d})")

    # Engage torque first AND seed Goal_Position to current pose so the
    # motor doesn't snap to a stale Goal_Position that might be set.
    for sid_str in target.keys():
        sid = int(sid_str)
        for _ in range(3):
            c, _ = pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
            if c == COMM_SUCCESS: break
    for sid_str in target.keys():
        sid = int(sid_str)
        for _ in range(3):
            c, _ = pkt.write2ByteTxRx(ph, sid, ADDR_GOAL_POSITION, start[sid_str])
            if c == COMM_SUCCESS: break

    n_steps = max(1, int(duration * rate))
    period = 1.0 / rate
    for step in range(1, n_steps + 1):
        t = step / n_steps
        for sid_str, end in target.items():
            sid = int(sid_str)
            interp = int(round((1.0 - t) * start[sid_str] + t * end))
            pkt.write2ByteTxRx(ph, sid, ADDR_GOAL_POSITION, interp)
        time.sleep(period)
    print(f"[{side}] reached home pose, holding (motors stay engaged)")
    ph.closePort()


def main() -> None:
    p = argparse.ArgumentParser(description="Drive SO-101 arms to saved home pose (raw scservo)")
    p.add_argument("--left-only", action="store_true")
    p.add_argument("--right-only", action="store_true")
    p.add_argument("--duration", type=float, default=4.0,
                   help="seconds to interpolate from current to home (default 4)")
    p.add_argument("--rate", type=float, default=30.0,
                   help="Hz tick during ramp (default 30)")
    p.add_argument("--pose", default=DEFAULT_POSE,
                   help=f"JSON file with home pose (default {DEFAULT_POSE})")
    args = p.parse_args()
    if args.left_only and args.right_only:
        p.error("--left-only and --right-only are mutually exclusive")

    if not os.path.exists(args.pose):
        p.error(
            f"home pose file not found: {args.pose}\n"
            f"Pose your arms manually to the position you want as 'home', "
            f"then snapshot it (see Configure step in README) - it writes "
            f"scripts/home_pose.json. An example is at {EXAMPLE_POSE}."
        )

    with open(args.pose) as f:
        pose = json.load(f)

    sides = []
    if not args.right_only: sides.append("left")
    if not args.left_only:  sides.append("right")

    interrupted = False
    def _on_sigint(*_):
        nonlocal interrupted
        interrupted = True
        print("\n[main] Ctrl+C - finishing current arm then stopping")
    signal.signal(signal.SIGINT, _on_sigint)

    for side in sides:
        if interrupted: break
        if side not in pose:
            print(f"[{side}] no pose entry in {args.pose}, skipping"); continue
        home_one_arm(side, PORTS[side], pose[side], args.duration, args.rate)


if __name__ == "__main__":
    main()

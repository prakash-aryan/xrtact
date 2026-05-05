"""Drive SO-101 arms to a saved home pose via lerobot SOFollower.

Note: home.py is the preferred replacement - it uses raw scservo, avoids
lerobot's flaky strict ping, and reads home_pose.json instead of the
per-arm dict below.
"""

from __future__ import annotations

import argparse
import signal
import time

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

_ADDR_TORQUE_ENABLE = 40
_ADDR_GOAL_POSITION = 42
_ADDR_PRESENT_POSITION = 56


def _force_torque_on(port_name: str) -> None:
    """Belt-and-suspenders re-engage torque via raw scservo, in case
    lerobot's torque_disabled context manager exited via an exception
    and left some motors disabled."""
    ph = PortHandler(port_name)
    if not ph.openPort() or not ph.setBaudRate(1_000_000):
        print(f"  [warn] {port_name}: could not open port to force torque on")
        return
    pkt = PacketHandler(0)
    for sid in range(1, 7):
        pos, comm, _ = pkt.read2ByteTxRx(ph, sid, _ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS:
            continue
        for _ in range(3):
            c1, _ = pkt.write2ByteTxRx(ph, sid, _ADDR_GOAL_POSITION, pos)
            if c1 == COMM_SUCCESS:
                break
        for _ in range(3):
            c2, _ = pkt.write1ByteTxRx(ph, sid, _ADDR_TORQUE_ENABLE, 1)
            if c2 == COMM_SUCCESS:
                break
    ph.closePort()


HOME_POSE_PER_ARM: dict[str, dict[str, float]] = {
    "left_follower": {
        "shoulder_pan.pos":  42.0,
        "shoulder_lift.pos":  0.0,
        "elbow_flex.pos":     0.0,
        "wrist_flex.pos":     0.0,
        "wrist_roll.pos":     0.0,
        "gripper.pos":        0.0,
    },
    "right_follower": {
        "shoulder_pan.pos":   5.0,
        "shoulder_lift.pos":  0.0,
        "elbow_flex.pos":     0.0,
        "wrist_flex.pos":     0.0,
        "wrist_roll.pos":     0.0,
        "gripper.pos":        0.0,
    },
}
HOME_POSE = HOME_POSE_PER_ARM["left_follower"]


def home_arm(port: str, arm_id: str, duration: float, rate: float, release: bool) -> None:
    cfg = SOFollowerRobotConfig(
        port=port,
        id=arm_id,
        use_degrees=True,
        disable_torque_on_disconnect=release,
    )
    arm = SOFollower(cfg)
    print(f"\n[{arm_id}] connecting on {port}")
    arm.connect()
    target_pose = HOME_POSE_PER_ARM.get(arm_id, HOME_POSE)
    try:
        start_obs = arm.get_observation()
        start = {k: float(start_obs[k]) for k in target_pose}
        print(f"[{arm_id}] start pose:")
        for k, v in start.items():
            print(f"    {k:<18s} {v:+8.2f} deg  (target {target_pose[k]:+8.2f})")

        n_steps = max(1, int(duration * rate))
        period = 1.0 / rate
        for step in range(1, n_steps + 1):
            t = step / n_steps
            target = {k: (1.0 - t) * start[k] + t * target_pose[k] for k in target_pose}
            arm.send_action(target)
            time.sleep(period)

        if release:
            print(f"[{arm_id}] reached home pose, releasing torque on disconnect")
        else:
            print(f"[{arm_id}] reached home pose, holding (motors stay engaged)")
    finally:
        arm.disconnect()
        if not release:
            _force_torque_on(port)


def main() -> None:
    p = argparse.ArgumentParser(description="Move SO-101 follower(s) to calibrated zero pose")
    p.add_argument("--left-port", default="/dev/ttyACM0")
    p.add_argument("--right-port", default="/dev/ttyACM1")
    p.add_argument("--left-only", action="store_true")
    p.add_argument("--right-only", action="store_true")
    p.add_argument(
        "--duration", type=float, default=3.0,
        help="seconds to interpolate from current to home (default 3)",
    )
    p.add_argument(
        "--rate", type=float, default=30.0,
        help="control rate in Hz during the ramp (default 30)",
    )
    p.add_argument(
        "--release", action="store_true",
        help="release motor torque on exit (default: keep motors engaged so arms hold the home pose)",
    )
    args = p.parse_args()

    if args.left_only and args.right_only:
        p.error("--left-only and --right-only are mutually exclusive")

    arms: list[tuple[str, str]] = []
    if not args.right_only:
        arms.append((args.left_port, "left_follower"))
    if not args.left_only:
        arms.append((args.right_port, "right_follower"))

    interrupted = False

    def _on_sigint(*_):
        nonlocal interrupted
        interrupted = True
        print("\n[main] Ctrl+C - finishing current arm then stopping")

    signal.signal(signal.SIGINT, _on_sigint)

    for port, arm_id in arms:
        if interrupted:
            print(f"[{arm_id}] skipped (interrupted)")
            continue
        try:
            home_arm(port, arm_id, args.duration, args.rate, release=args.release)
        except Exception as e:
            print(f"[{arm_id}] FAILED: {e}")


if __name__ == "__main__":
    main()

"""home_arms.py - move SO-101 follower arm(s) to their calibrated zero pose.

Connects via the lerobot SOFollower API (so the calibration we saved at
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/{id}.json` is
applied automatically), snapshots the current joint angles, then linearly
interpolates each joint from where it is now to 0 deg over `--duration`
seconds at `--rate` Hz. Disconnects cleanly so the motors release torque.

Usage (from the xrtact venv that has lerobot + scservo_sdk available):
    python scripts/home_arms.py                      # both arms, 3 s ramp
    python scripts/home_arms.py --left-only          # one arm only
    python scripts/home_arms.py --duration 5 --rate 30
"""

from __future__ import annotations

import argparse
import signal
import time

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig


# Calibrated home = all joints at 0 deg (the per-joint mid of the swept
# range_min/range_max captured during `lerobot-calibrate`).
HOME_POSE: dict[str, float] = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": 0.0,
    "elbow_flex.pos": 0.0,
    "wrist_flex.pos": 0.0,
    "wrist_roll.pos": 0.0,
    "gripper.pos": 0.0,
}


def home_arm(port: str, arm_id: str, duration: float, rate: float, release: bool) -> None:
    # disable_torque_on_disconnect controls whether SOFollower turns motors off
    # when we disconnect at the end. Default (release=False) leaves motors
    # engaged so the arms HOLD the home pose; otherwise they fall under gravity
    # the instant the script exits.
    cfg = SOFollowerRobotConfig(
        port=port,
        id=arm_id,
        use_degrees=True,
        disable_torque_on_disconnect=release,
    )
    arm = SOFollower(cfg)
    print(f"\n[{arm_id}] connecting on {port}")
    arm.connect()
    try:
        start_obs = arm.get_observation()
        start = {k: float(start_obs[k]) for k in HOME_POSE}
        print(f"[{arm_id}] start pose:")
        for k, v in start.items():
            print(f"    {k:<18s} {v:+8.2f} deg")

        n_steps = max(1, int(duration * rate))
        period = 1.0 / rate
        for step in range(1, n_steps + 1):
            t = step / n_steps
            target = {k: (1.0 - t) * start[k] + t * HOME_POSE[k] for k in HOME_POSE}
            arm.send_action(target)
            time.sleep(period)

        if release:
            print(f"[{arm_id}] reached home pose, releasing torque on disconnect")
        else:
            print(f"[{arm_id}] reached home pose, holding (motors stay engaged)")
    finally:
        arm.disconnect()


def main() -> None:
    p = argparse.ArgumentParser(description="Move SO-101 follower(s) to calibrated zero pose")
    p.add_argument("--left-port", default="/dev/ttyACM1")
    p.add_argument("--right-port", default="/dev/ttyACM0")
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

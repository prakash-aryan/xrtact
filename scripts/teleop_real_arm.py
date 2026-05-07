"""Subscribe to telegrip's ZMQ joint-target stream and drive ONE SO-101 arm.

No cameras, no dataset writes - just teleop relay for debugging.
"""

from __future__ import annotations

import argparse
import json
import signal
import time

import numpy as np
import zmq

from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

JOINT_NAMES = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]
ARM_PORTS = {"left": "/dev/ttyACM0", "right": "/dev/ttyACM1"}
ARM_IDS = {"right": "right_follower", "left": "left_follower"}

_ADDR_TORQUE_ENABLE = 40
_ADDR_GOAL_POSITION = 42
_ADDR_PRESENT_POSITION = 56


def _force_torque_on(port_name: str) -> None:
    """Set Goal_Position=current then Torque_Enable=1 to defeat lerobot's
    brief connect-time torque drop."""
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


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--arm", choices=["right", "left"], default="right")
    p.add_argument("--zmq-endpoint", default="tcp://localhost:5555")
    p.add_argument("--fps", type=int, default=30)
    args = p.parse_args()

    interrupted = False

    def _on_sigint(*_):
        nonlocal interrupted
        interrupted = True
        print("\n[teleop] Ctrl+C - stopping")

    signal.signal(signal.SIGINT, _on_sigint)

    arm_id = ARM_IDS[args.arm]
    arm_port = ARM_PORTS[args.arm]
    print(f"[teleop] connecting {arm_id} on {arm_port}")
    arm = SOFollower(SOFollowerRobotConfig(
        port=arm_port, id=arm_id, use_degrees=True,
        disable_torque_on_disconnect=False,
    ))
    arm.connect()
    _force_torque_on(arm_port)

    print(f"[teleop] subscribing to telegrip on {args.zmq_endpoint}")
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(args.zmq_endpoint)

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    period = 1.0 / args.fps
    MAX_DEG_PER_SEC = 90.0
    max_step = MAX_DEG_PER_SEC * period

    # Anchor: capture arm pose + first ZMQ target so we command
    # arm_cmd = arm_init + (zmq_target - zmq_init). Tracks controller deltas
    # instead of absolute IK output, since telegrip's IK frame doesn't match
    # our motor calibration frame.
    obs = arm.get_observation()
    arm_init = np.array([float(obs[k]) for k in JOINT_NAMES], dtype=np.float32)
    cmd = arm_init.copy()
    zmq_init = None

    print(f"[teleop] arm_init = "
          f"[{arm_init[0]:+.1f}, {arm_init[1]:+.1f}, {arm_init[2]:+.1f}, "
          f"{arm_init[3]:+.1f}, {arm_init[4]:+.1f}, {arm_init[5]:+.1f}]")
    print(f"[teleop] driving the {args.arm.upper()} arm. Move it with the Quest, Ctrl+C to end.")

    n = 0
    try:
        while not interrupted:
            tick0 = time.time()
            new_targets = None
            while True:
                events = dict(poller.poll(timeout=0))
                if sock not in events:
                    break
                try:
                    raw = sock.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break
                try:
                    msg = json.loads(raw)
                    new_targets = msg.get(args.arm)
                except Exception as e:
                    print(f"[teleop] bad message: {e}")

            if new_targets is not None:
                zmq_target = np.asarray(new_targets, dtype=np.float32)
                if zmq_init is None:
                    zmq_init = zmq_target.copy()
                    print(f"[teleop] zmq_init (anchor) = "
                          f"[{zmq_init[0]:+.1f}, {zmq_init[1]:+.1f}, {zmq_init[2]:+.1f}, "
                          f"{zmq_init[3]:+.1f}, {zmq_init[4]:+.1f}, {zmq_init[5]:+.1f}]")
                desired = arm_init + (zmq_target - zmq_init)
            else:
                desired = cmd

            delta = desired - cmd
            delta = np.clip(delta, -max_step, max_step)
            cmd = cmd + delta

            action_dict = dict(zip(JOINT_NAMES, cmd.tolist()))
            try:
                arm.send_action(action_dict)
            except Exception as e:
                print(f"[teleop] send_action error: {e}")

            n += 1
            if n % 30 == 1:
                z_str = "no zmq yet" if zmq_init is None else (
                    f"des[0..2]=[{desired[0]:+.1f},{desired[1]:+.1f},{desired[2]:+.1f}]"
                )
                print(f"  n={n:>5}  cmd[0..2]=[{cmd[0]:+.1f},{cmd[1]:+.1f},{cmd[2]:+.1f}]  "
                      f"grip={cmd[5]:+.1f}  {z_str}")

            elapsed = time.time() - tick0
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        try:
            sock.close(0)
        except Exception:
            pass
        try:
            arm.disconnect()
        except Exception as e:
            print(f"[teleop] arm disconnect: {e}")
        print("[teleop] done")


if __name__ == "__main__":
    main()

"""record_real_arm.py - Quest teleop -> real SO-101 -> LeRobotDataset.

Architecture
------------
Telegrip (run separately with `--no-robot --log-level info`) reads the
Quest 3 controller poses, runs IK, and publishes joint targets as JSON
on tcp://*:5555:
    {"left":[6 floats deg], "right":[6 floats deg]}

This script SUBSCRIBES to that stream, drives ONE real SO-101 arm, reads
back joint state + camera frames, and writes each tick to a LeRobotDataset
in the standard schema. Pressing Ctrl+C ends the current episode, marks
it successful, and finalises the dataset.

Single-arm only for now (SmolVLA / pi0 baseline are single-arm); easy to
extend to bimanual later by writing 12-dim state/action and connecting
both followers.

Usage
-----
    # terminal 1 - telegrip publishes joint targets (no real arms attached)
    cd ~/telegrip
    .venv/bin/python -u -m telegrip --no-robot --log-level info

    # terminal 2 - this recorder drives the real arm + writes dataset
    /home/merabro/telegrip/.venv/bin/python \
        /home/merabro/xrtact/scripts/record_real_arm.py \
        --repo-id local/realarm_pickplace_smoke \
        --arm right \
        --max-episode-s 60

Output
------
~/.cache/huggingface/lerobot/<repo_id>/  in standard LeRobotDataset format.
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from dataclasses import asdict, dataclass

import cv2
import numpy as np
import torch
import zmq

from lerobot.cameras.opencv.camera_opencv import OpenCVCamera
from lerobot.cameras.opencv.configuration_opencv import (
    Cv2Backends,
    OpenCVCameraConfig,
)
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

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
REALSENSE_SERIAL = "317622071570"
WRIST_CAM_INDEX = 6  # /dev/video2 - the Logitech that opens after RealSense connects


def make_features(image_h: int, image_w: int) -> dict:
    """LeRobotDataset feature schema for single-arm SO-101 + 2 cameras.

    Schema names follow the standardised lerobot convention so the
    dataset can be fed to ACT / SmolVLA / pi0 fine-tuning later with at
    most a small camera-name remap.
    """
    state_or_action = {
        "dtype": "float32",
        "shape": (6,),
        "names": JOINT_NAMES,
    }
    image = {
        "dtype": "video",
        "shape": (image_h, image_w, 3),
        "names": ["height", "width", "channels"],
    }
    return {
        "observation.state": state_or_action,
        "action": state_or_action,
        "observation.images.top": image,
        "observation.images.wrist": image,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-id", required=True,
                        help="dataset repo id, e.g. local/realarm_pickplace_smoke")
    parser.add_argument("--arm", choices=["right", "left"], default="right")
    parser.add_argument("--task", default="Pick up the object on the table.",
                        help="Natural-language task description per LeRobot convention.")
    parser.add_argument("--zmq-endpoint", default="tcp://localhost:5555",
                        help="Telegrip's joint-target publisher endpoint.")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--max-episode-s", type=float, default=60.0,
                        help="Hard cap on a single episode (Ctrl+C to end earlier).")
    parser.add_argument("--cam-w", type=int, default=640)
    parser.add_argument("--cam-h", type=int, default=480)
    args = parser.parse_args()

    interrupted = False

    def _on_sigint(*_):
        nonlocal interrupted
        interrupted = True
        print("\n[record] Ctrl+C - finishing episode and saving...")

    signal.signal(signal.SIGINT, _on_sigint)

    # 1. Cameras (RealSense first - that's the order pyrealsense2 needs)
    print("[1/4] connecting RealSense (top)...")
    rs = RealSenseCamera(RealSenseCameraConfig(
        serial_number_or_name=REALSENSE_SERIAL,
        fps=args.fps, width=args.cam_w, height=args.cam_h,
    ))
    rs.connect()
    print("[2/4] connecting Logitech wrist (/dev/video{0})...".format(WRIST_CAM_INDEX))
    wrist_cam = OpenCVCamera(OpenCVCameraConfig(
        index_or_path=WRIST_CAM_INDEX, backend=Cv2Backends.V4L2,
        fps=args.fps, width=args.cam_w, height=args.cam_h,
    ))
    wrist_cam.connect()

    # 2. Real arm (single arm for now)
    arm_id = ARM_IDS[args.arm]
    arm_port = ARM_PORTS[args.arm]
    print(f"[3/4] connecting {arm_id} on {arm_port}...")
    arm = SOFollower(SOFollowerRobotConfig(
        port=arm_port, id=arm_id, use_degrees=True,
        disable_torque_on_disconnect=False,  # keep arm holding pose at end
    ))
    arm.connect()

    # 3. ZMQ subscriber for telegrip's joint-target stream
    print(f"[4/4] subscribing to telegrip on {args.zmq_endpoint}...")
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)  # always read latest, drop stale
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(args.zmq_endpoint)

    # 4. Dataset
    features = make_features(args.cam_h, args.cam_w)
    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        robot_type="so101_follower",
        features=features,
        use_videos=True,
    )
    print(f"[record] dataset will be written to ~/.cache/huggingface/lerobot/{args.repo_id}/")
    print(f"[record] task: {args.task!r}")
    print(f"[record] driving the {args.arm.upper()} arm. Move it with the Quest, Ctrl+C to end.")

    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    period = 1.0 / args.fps
    start_t = time.time()
    n_frames = 0
    last_action = None  # 6-d numpy float32 of last commanded action

    try:
        while not interrupted and (time.time() - start_t) < args.max_episode_s:
            tick0 = time.time()

            # Drain any pending ZMQ messages, keep the latest
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
                    new_targets = msg.get(args.arm)  # list of 6 floats (deg)
                except Exception as e:
                    print(f"[record] bad message: {e}")

            if new_targets is not None:
                last_action = np.asarray(new_targets, dtype=np.float32)
                action_dict = dict(zip(JOINT_NAMES, last_action.tolist()))
                try:
                    arm.send_action(action_dict)
                except Exception as e:
                    print(f"[record] send_action error: {e}")

            # Skip writing frames until we've received at least one action
            if last_action is None:
                time.sleep(period)
                continue

            # Read current observation
            obs = arm.get_observation()
            state = np.array([float(obs[k]) for k in JOINT_NAMES], dtype=np.float32)
            top = rs.async_read()
            wrist = wrist_cam.async_read()

            # Sanity: reject malformed frames so the dataset writer doesn't crash
            if top is None or wrist is None:
                continue
            if top.shape[:2] != (args.cam_h, args.cam_w):
                top = cv2.resize(top, (args.cam_w, args.cam_h))
            if wrist.shape[:2] != (args.cam_h, args.cam_w):
                wrist = cv2.resize(wrist, (args.cam_w, args.cam_h))

            ds.add_frame(
                {
                    "observation.state": state,
                    "action": last_action,
                    "observation.images.top": top,
                    "observation.images.wrist": wrist,
                },
                task=args.task,
            )
            n_frames += 1
            if n_frames % 30 == 1:
                print(f"  t={time.time()-start_t:5.2f}s  frame={n_frames:>4}  "
                      f"state[0..2]=[{state[0]:+.1f}, {state[1]:+.1f}, {state[2]:+.1f}]  "
                      f"act[0..2]=[{last_action[0]:+.1f}, {last_action[1]:+.1f}, {last_action[2]:+.1f}]")

            elapsed = time.time() - tick0
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        print(f"\n[record] stopping. captured {n_frames} frames in "
              f"{time.time()-start_t:.1f}s.")
        # Save episode + finalise dataset (single episode for the smoke test)
        try:
            if n_frames > 0:
                ds.save_episode()
                print(f"[record] episode saved")
            ds.finalize()
            print(f"[record] dataset finalized")
        except Exception as e:
            print(f"[record] dataset save error: {e}")

        try:
            sock.close(0)
        except Exception:
            pass
        try:
            arm.disconnect()
        except Exception as e:
            print(f"[record] arm disconnect: {e}")
        try:
            wrist_cam.disconnect()
        except Exception as e:
            print(f"[record] wrist cam disconnect: {e}")
        try:
            rs.disconnect()
        except Exception as e:
            print(f"[record] realsense disconnect: {e}")
        print("[record] done")


if __name__ == "__main__":
    main()

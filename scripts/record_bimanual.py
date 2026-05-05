"""Quest teleop -> both SO-101 arms -> LeRobotDataset (LeHome schema)."""

from __future__ import annotations

import argparse
import json
import os
import signal
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import zmq
from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

from lerobot.datasets.lerobot_dataset import LeRobotDataset

LEFT_PORT = os.environ.get("XRTACT_LEFT_PORT", "/dev/ttyACM0")
RIGHT_PORT = os.environ.get("XRTACT_RIGHT_PORT", "/dev/ttyACM1")
LEFT_CAM_INDEX = int(os.environ.get("XRTACT_LEFT_CAM_INDEX", "10"))
RIGHT_CAM_INDEX = int(os.environ.get("XRTACT_RIGHT_CAM_INDEX", "8"))
REALSENSE_SERIAL = os.environ.get("XRTACT_REALSENSE_SERIAL", "")
if not REALSENSE_SERIAL:
    raise RuntimeError(
        "Set XRTACT_REALSENSE_SERIAL to your RealSense's serial number "
        "(find it via `rs-enumerate-devices | grep Serial`)."
    )

JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
CALIB_DIR = os.path.expanduser(
    "~/.cache/huggingface/lerobot/calibration/robots/so_follower"
)

ADDR_PRESENT_POSITION = 56
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42

# From LeIsaac SO101_FOLLOWER_USD_JOINT_LIMITS - the policy was trained
# against these so we map our motor counts into this range.
URDF_JOINT_LIMITS = {
    "shoulder_pan":  (-110.0, 110.0),
    "shoulder_lift": (-100.0, 100.0),
    "elbow_flex":    (-100.0,  90.0),
    "wrist_flex":     (-95.0,  95.0),
    "wrist_roll":   (-160.0, 160.0),
    "gripper":        (-10.0, 100.0),
}

# Sign per joint to align motor count direction with URDF positive direction.
# Empirical - flip a joint if controller motion produces inverted real motion.
INVERT = {
    "left": {
        "shoulder_pan":  -1,
        "shoulder_lift": -1,
        "elbow_flex":    +1,
        "wrist_flex":    +1,
        "wrist_roll":    +1,
        "gripper":       -1,
    },
    "right": {
        "shoulder_pan":  -1,
        "shoulder_lift": -1,
        "elbow_flex":    +1,
        "wrist_flex":    +1,
        "wrist_roll":    +1,
        "gripper":       -1,
    },
}

WIDTH, HEIGHT = 640, 480
FPS = 30


def raw_to_urdf_rad(raw: int, jname: str, calib_entry: dict, side: str) -> float:
    rmin, rmax = calib_entry["range_min"], calib_entry["range_max"]
    jmin, jmax = URDF_JOINT_LIMITS[jname]
    sign = INVERT[side][jname]
    if sign > 0:
        frac = (raw - rmin) / (rmax - rmin)
    else:
        frac = (rmax - raw) / (rmax - rmin)
    return float(np.radians(jmin + frac * (jmax - jmin)))


def urdf_rad_to_raw(rad: float, jname: str, calib_entry: dict, side: str) -> int:
    rmin, rmax = calib_entry["range_min"], calib_entry["range_max"]
    jmin, jmax = URDF_JOINT_LIMITS[jname]
    urdf_deg = float(np.degrees(rad))
    frac = (urdf_deg - jmin) / (jmax - jmin)
    sign = INVERT[side][jname]
    if sign > 0:
        raw = rmin + frac * (rmax - rmin)
    else:
        raw = rmax - frac * (rmax - rmin)
    return int(round(np.clip(raw, rmin, rmax)))


def lerobot_value_to_urdf_rad(val: float, jname: str, calib_entry: dict, side: str) -> float:
    homing = calib_entry["homing_offset"]
    if jname == "gripper":
        # Telegrip publishes 0=open / 45=closed; lerobot RANGE_0_100 has
        # 0=range_min=closed-on-this-rig. Inverted, hence (45-val).
        rmin, rmax = calib_entry["range_min"], calib_entry["range_max"]
        pct = float(np.clip((45.0 - val) / 45.0 * 100.0, 0.0, 100.0))
        raw = int(round(rmin + pct / 100.0 * (rmax - rmin)))
    else:
        raw = int(round(homing + val / 360.0 * 4096))
    return raw_to_urdf_rad(raw, jname, calib_entry, side)


def load_calibrations() -> dict:
    out = {}
    for side in ("left", "right"):
        with open(f"{CALIB_DIR}/{side}_follower.json") as f:
            out[side] = json.load(f)
    return out


def open_arm(port: str) -> tuple[PortHandler, PacketHandler]:
    ph = PortHandler(port)
    if not (ph.openPort() and ph.setBaudRate(1_000_000)):
        raise RuntimeError(f"could not open {port}")
    return ph, PacketHandler(0)


def force_torque_on(port_name: str) -> None:
    """Set Goal_Position = current then Torque_Enable = 1.

    Counters lerobot's brief connect-time torque drop that lets gravity
    drop the arm.
    """
    ph = PortHandler(port_name)
    if not (ph.openPort() and ph.setBaudRate(1_000_000)):
        return
    pkt = PacketHandler(0)
    for sid in range(1, 7):
        pos, comm, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
        if comm != COMM_SUCCESS:
            continue
        for _ in range(3):
            c, _ = pkt.write2ByteTxRx(ph, sid, ADDR_GOAL_POSITION, pos)
            if c == COMM_SUCCESS:
                break
        for _ in range(3):
            c, _ = pkt.write1ByteTxRx(ph, sid, ADDR_TORQUE_ENABLE, 1)
            if c == COMM_SUCCESS:
                break
    ph.closePort()


def read_arm_state(
    ph: PortHandler, pkt: PacketHandler, calib: dict, side: str,
) -> np.ndarray:
    out = np.zeros(6, dtype=np.float32)
    for i, jname in enumerate(JOINTS):
        sid = calib[jname]["id"]
        raw = None
        for _ in range(5):
            r, comm, _ = pkt.read2ByteTxRx(ph, sid, ADDR_PRESENT_POSITION)
            if comm == COMM_SUCCESS:
                raw = r
                break
        if raw is None:
            raise RuntimeError(f"could not read motor id={sid}")
        out[i] = raw_to_urdf_rad(raw, jname, calib[jname], side)
    return out


def write_arm_action(
    ph: PortHandler, pkt: PacketHandler, calib: dict, action_rad: np.ndarray, side: str,
) -> None:
    for i, jname in enumerate(JOINTS):
        raw_target = urdf_rad_to_raw(float(action_rad[i]), jname, calib[jname], side)
        for _ in range(3):
            c, _ = pkt.write2ByteTxRx(
                ph, calib[jname]["id"], ADDR_GOAL_POSITION, raw_target
            )
            if c == COMM_SUCCESS:
                break


def open_v4l2(idx: int) -> cv2.VideoCapture:
    """Open at MJPG; caller must defer .read() until ALL Logitechs are open
    (reading right after opening one starves the other on the same controller)."""
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    if not cap.isOpened():
        raise RuntimeError(f"could not open /dev/video{idx}")
    time.sleep(0.3)
    return cap


def warmup_v4l2(cap: cv2.VideoCapture, idx: int) -> None:
    for _ in range(10):
        ok, frame = cap.read()
        if ok and frame is not None:
            return
    raise RuntimeError(f"/dev/video{idx} produced no frames after warmup")


def grab_v4l2_rgb(cap: cv2.VideoCapture) -> np.ndarray:
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("v4l2 grab failed")
    if frame.shape[:2] != (HEIGHT, WIDTH):
        frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
    return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)


def open_realsense() -> rs.pipeline:
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(REALSENSE_SERIAL)
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
    pipe.start(cfg)
    for _ in range(5):
        pipe.wait_for_frames(timeout_ms=2000)
    return pipe


def grab_realsense_rgb(pipe: rs.pipeline) -> np.ndarray:
    frames = pipe.wait_for_frames(timeout_ms=2000)
    arr = np.asanyarray(frames.get_color_frame().get_data())
    if arr.shape[:2] != (HEIGHT, WIDTH):
        arr = cv2.resize(arr, (WIDTH, HEIGHT))
    return arr


def make_features() -> dict:
    state_or_action = {
        "dtype": "float32",
        "shape": (12,),
        "names": [f"left_{j}" for j in JOINTS] + [f"right_{j}" for j in JOINTS],
    }
    image = {
        "dtype": "video",
        "shape": (HEIGHT, WIDTH, 3),
        "names": ["height", "width", "channels"],
    }
    return {
        "observation.state": state_or_action,
        "action": state_or_action,
        "observation.images.top_rgb": image,
        "observation.images.left_rgb": image,
        "observation.images.right_rgb": image,
    }


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--repo-id", required=True)
    p.add_argument("--task", default="Fold the full-sleeve shirt.")
    p.add_argument("--zmq-endpoint", default="tcp://localhost:5555")
    p.add_argument("--fps", type=int, default=FPS)
    p.add_argument("--max-episode-s", type=float, default=60.0)
    args = p.parse_args()

    interrupted = False

    def _on_sigint(*_):
        nonlocal interrupted
        interrupted = True
        print("\n[record] Ctrl+C - finishing episode and saving...")

    signal.signal(signal.SIGINT, _on_sigint)

    cal = load_calibrations()

    print(f"[1/3] connecting RealSense (serial {REALSENSE_SERIAL})...")
    rs_pipe = open_realsense()
    print(f"[1/3] connecting LEFT wrist /dev/video{LEFT_CAM_INDEX}...")
    left_cam = open_v4l2(LEFT_CAM_INDEX)
    print(f"[1/3] connecting RIGHT wrist /dev/video{RIGHT_CAM_INDEX}...")
    right_cam = open_v4l2(RIGHT_CAM_INDEX)
    print("[1/3] warming up both wrist cams...")
    warmup_v4l2(left_cam, LEFT_CAM_INDEX)
    warmup_v4l2(right_cam, RIGHT_CAM_INDEX)

    print(f"[2/3] opening LEFT arm {LEFT_PORT}...")
    left_ph, left_pkt = open_arm(LEFT_PORT)
    print(f"[2/3] opening RIGHT arm {RIGHT_PORT}...")
    right_ph, right_pkt = open_arm(RIGHT_PORT)
    print("[2/3] engaging torque on both arms (hold current pose)...")
    force_torque_on(LEFT_PORT)
    force_torque_on(RIGHT_PORT)

    print(f"[3/3] subscribing to telegrip on {args.zmq_endpoint}...")
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.setsockopt(zmq.CONFLATE, 1)
    sock.setsockopt(zmq.LINGER, 0)
    sock.connect(args.zmq_endpoint)
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    features = make_features()
    ds_root = os.path.expanduser(f"~/.cache/huggingface/lerobot/{args.repo_id}")
    if os.path.exists(ds_root):
        raise RuntimeError(
            f"Dataset path already exists at {ds_root}. Pick a different "
            f"--repo-id or `rm -rf {ds_root}` first."
        )
    print(f"[record] creating new dataset at {ds_root}")
    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=args.fps,
        robot_type="so101_bimanual",
        features=features,
        use_videos=True,
    )
    print(f"[record] task: {args.task!r}")
    print("[record] In VR press X (left) + A (right) together to start/stop "
          "recording. Ctrl+C ends episode.")

    period = 1.0 / args.fps
    MAX_DEG_PER_SEC = 90.0
    max_step_rad = float(np.radians(MAX_DEG_PER_SEC) * period)

    start_t = time.time()
    n_frames = 0
    last_target = None
    last_recording_flag = False

    # When recording flag rises we capture (arm_anchor, zmq_anchor) and from
    # then on command arm = arm_anchor + (zmq_target - zmq_anchor). This
    # tracks controller motion deltas instead of absolute IK output, which
    # avoids slamming because telegrip's IK frame doesn't match our motor
    # calibration frame.
    arm_anchor = None
    zmq_anchor = None

    l0 = read_arm_state(left_ph, left_pkt, cal["left"], "left")
    r0 = read_arm_state(right_ph, right_pkt, cal["right"], "right")
    cmd_rad = np.concatenate([l0, r0]).astype(np.float32)

    try:
        while not interrupted and (time.time() - start_t) < args.max_episode_s:
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
                    new_targets = json.loads(raw)
                except Exception as e:
                    print(f"[record] bad message: {e}")

            if new_targets is not None:
                left_deg = new_targets.get("left")
                right_deg = new_targets.get("right")
                if (left_deg is not None and right_deg is not None
                        and len(left_deg) == 6 and len(right_deg) == 6):
                    last_target = np.array(
                        [lerobot_value_to_urdf_rad(left_deg[i], j, cal["left"][j], "left")
                         for i, j in enumerate(JOINTS)]
                        + [lerobot_value_to_urdf_rad(right_deg[i], j, cal["right"][j], "right")
                           for i, j in enumerate(JOINTS)],
                        dtype=np.float32,
                    )
                rec = bool(new_targets.get("recording", False))
                if rec != last_recording_flag:
                    print(f"[record] recording flag -> {rec}")
                    if rec and last_target is not None:
                        l_state = read_arm_state(left_ph, left_pkt, cal["left"], "left")
                        r_state = read_arm_state(right_ph, right_pkt, cal["right"], "right")
                        arm_anchor = np.concatenate([l_state, r_state]).astype(np.float32)
                        zmq_anchor = last_target.copy()
                last_recording_flag = rec

            if last_target is None:
                time.sleep(period)
                continue

            if not last_recording_flag:
                cmd_rad = np.concatenate([
                    read_arm_state(left_ph, left_pkt, cal["left"], "left"),
                    read_arm_state(right_ph, right_pkt, cal["right"], "right"),
                ]).astype(np.float32)
                time.sleep(period)
                continue

            if arm_anchor is not None and zmq_anchor is not None:
                desired = arm_anchor + (last_target - zmq_anchor)
            else:
                desired = last_target

            delta = desired - cmd_rad
            delta = np.clip(delta, -max_step_rad, max_step_rad)
            cmd_rad = cmd_rad + delta

            try:
                write_arm_action(left_ph, left_pkt, cal["left"], cmd_rad[:6], "left")
                write_arm_action(right_ph, right_pkt, cal["right"], cmd_rad[6:], "right")
            except Exception as e:
                print(f"[record] write_arm_action error: {e}")

            l_state = read_arm_state(left_ph, left_pkt, cal["left"], "left")
            r_state = read_arm_state(right_ph, right_pkt, cal["right"], "right")
            state12 = np.concatenate([l_state, r_state]).astype(np.float32)

            top = grab_realsense_rgb(rs_pipe)
            lf = grab_v4l2_rgb(left_cam)
            rf = grab_v4l2_rgb(right_cam)

            ds.add_frame({
                "observation.state": state12,
                "action": cmd_rad.copy(),
                "observation.images.top_rgb": top,
                "observation.images.left_rgb": lf,
                "observation.images.right_rgb": rf,
                "task": args.task,
            })
            n_frames += 1
            if n_frames % 30 == 1:
                print(
                    f"  t={time.time()-start_t:5.2f}s  frame={n_frames:>4}  "
                    f"L_lift_obs={l_state[1]:+.2f}  L_lift_cmd={cmd_rad[1]:+.2f}  L_lift_tgt={last_target[1]:+.2f}  "
                    f"R_lift_obs={r_state[1]:+.2f}  R_lift_cmd={cmd_rad[7]:+.2f}"
                )

            elapsed = time.time() - tick0
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        print(
            f"\n[record] stopping. captured {n_frames} frames in "
            f"{time.time()-start_t:.1f}s."
        )
        try:
            if n_frames > 0:
                ds.save_episode()
                print("[record] episode saved")
            ds.finalize()
            print("[record] dataset finalized")
        except Exception as e:
            print(f"[record] dataset save error: {e}")

        try:
            sock.close(0)
        except Exception:
            pass
        try:
            rs_pipe.stop()
        except Exception:
            pass
        for cap in (left_cam, right_cam):
            try:
                cap.release()
            except Exception:
                pass
        for ph in (left_ph, right_ph):
            try:
                ph.closePort()
            except Exception:
                pass
        print("[record] done")


if __name__ == "__main__":
    main()

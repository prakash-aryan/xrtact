"""Client for the LeHome policy server: reads state + 3 cameras, POSTs
/infer, optionally writes the returned action back to the real arms."""

from __future__ import annotations

import base64
import json
import os
import signal
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import requests
from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler
from lerobot.robots.so_follower import SOFollower, SOFollowerRobotConfig

DRY_RUN = False
POLICY_URL = "http://localhost:8080"

SOFT_START_SECONDS = 3.0
MAX_DEG_PER_SEC = 180.0
N_STEPS = 600

LEFT_PORT = os.environ.get("XRTACT_LEFT_PORT", "/dev/ttyACM0")
RIGHT_PORT = os.environ.get("XRTACT_RIGHT_PORT", "/dev/ttyACM1")
LEFT_CAM_INDEX = int(os.environ.get("XRTACT_LEFT_CAM_INDEX", "4"))
RIGHT_CAM_INDEX = int(os.environ.get("XRTACT_RIGHT_CAM_INDEX", "6"))
REALSENSE_SERIAL = os.environ.get("XRTACT_REALSENSE_SERIAL", "317622071570")
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

WIDTH, HEIGHT = 640, 480
FPS = 30


# ---- State reading --------------------------------------------------------


def load_calibrations() -> dict:
    """Load left and right calibration JSONs."""
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


def raw_to_urdf_rad(raw: int, jname: str, calib_entry: dict) -> float:
    rmin, rmax = calib_entry["range_min"], calib_entry["range_max"]
    jmin, jmax = URDF_JOINT_LIMITS[jname]
    frac = (raw - rmin) / (rmax - rmin)
    return float(np.radians(jmin + frac * (jmax - jmin)))


def urdf_rad_to_raw(rad: float, jname: str, calib_entry: dict) -> int:
    rmin, rmax = calib_entry["range_min"], calib_entry["range_max"]
    jmin, jmax = URDF_JOINT_LIMITS[jname]
    urdf_deg = float(np.degrees(rad))
    frac = (urdf_deg - jmin) / (jmax - jmin)
    raw = rmin + frac * (rmax - rmin)
    return int(round(np.clip(raw, rmin, rmax)))


def read_arm_state(
    ph: PortHandler, pkt: PacketHandler, calib: dict
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
        out[i] = raw_to_urdf_rad(raw, jname, calib[jname])
    return out


def write_arm_action(
    ph: PortHandler, pkt: PacketHandler, calib: dict, action_rad: np.ndarray,
) -> None:
    for i, jname in enumerate(JOINTS):
        raw_target = urdf_rad_to_raw(float(action_rad[i]), jname, calib[jname])
        for _ in range(3):
            c, _ = pkt.write2ByteTxRx(
                ph, calib[jname]["id"], ADDR_GOAL_POSITION, raw_target
            )
            if c == COMM_SUCCESS:
                break


def open_v4l2(idx: int) -> cv2.VideoCapture:
    """Open at MJPG 640x480. Two C270s on the same USB controller break
    because they share a USB serial; mount on different controllers."""
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
    raise RuntimeError(
        f"/dev/video{idx} opened but produced no frames "
        f"(USB bandwidth conflict?)"
    )


def grab_v4l2_rgb(cap: cv2.VideoCapture) -> np.ndarray:
    ok, frame = cap.read()
    if not ok or frame is None:
        raise RuntimeError("v4l2 grab failed")
    if frame.shape[:2] != (HEIGHT, WIDTH):
        frame = cv2.resize(
            frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR
        )
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
    color = frames.get_color_frame()
    arr = np.asanyarray(color.get_data())
    if arr.shape[:2] != (HEIGHT, WIDTH):
        arr = cv2.resize(arr, (WIDTH, HEIGHT))
    return arr


def encode_image(arr: np.ndarray) -> dict:
    return {
        "base64": base64.b64encode(arr.tobytes()).decode("ascii"),
        "shape": list(arr.shape),
        "dtype": str(arr.dtype),
    }


def _force_torque_on(port_name: str) -> None:
    """Set Goal_Position=current then Torque_Enable=1 to defeat lerobot's
    brief connect-time torque drop."""
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


def _patch_feetech_ping_retries() -> None:
    """Monkey-patch FeetechMotorsBus.ping to default num_retry=5 instead of 0.

    Without this, the strict handshake aborts with 'missing motor id=N' on
    any single dropped packet, even when the motor is fine.
    """
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    if getattr(FeetechMotorsBus, "_ping_retry_patched", False):
        return
    orig_ping = FeetechMotorsBus.ping
    def patched(self, motor, num_retry=5, raise_on_error=False):
        return orig_ping(self, motor, num_retry=num_retry, raise_on_error=raise_on_error)
    FeetechMotorsBus.ping = patched
    FeetechMotorsBus._ping_retry_patched = True


def connect_arm(port: str, arm_id: str) -> SOFollower:
    _patch_feetech_ping_retries()
    arm = SOFollower(SOFollowerRobotConfig(
        port=port, id=arm_id, use_degrees=True,
        disable_torque_on_disconnect=False,
    ))
    arm.connect()
    _force_torque_on(port)
    return arm


def main() -> None:
    interrupted = False

    def _on_sigint(*_):
        nonlocal interrupted
        interrupted = True
        print("\n[client] Ctrl+C - stopping")

    signal.signal(signal.SIGINT, _on_sigint)

    print(f"[client] DRY_RUN={DRY_RUN}, policy={POLICY_URL}")

    cal = load_calibrations()

    print("[client] opening arms...")
    left_ph, left_pkt = open_arm(LEFT_PORT)
    right_ph, right_pkt = open_arm(RIGHT_PORT)

    # Open RealSense first then both Logitechs without reads in between -
    # reading from one Logitech right after opening starves the other.
    print(f"[client] opening RealSense (serial {REALSENSE_SERIAL})...")
    rs_pipe = open_realsense()
    print(f"[client] opening LEFT wrist /dev/video{LEFT_CAM_INDEX}...")
    left_cam = open_v4l2(LEFT_CAM_INDEX)
    print(f"[client] opening RIGHT wrist /dev/video{RIGHT_CAM_INDEX}...")
    right_cam = open_v4l2(RIGHT_CAM_INDEX)
    print("[client] warming up both wrist cams...")
    warmup_v4l2(left_cam, LEFT_CAM_INDEX)
    warmup_v4l2(right_cam, RIGHT_CAM_INDEX)

    if not DRY_RUN:
        print("[client] DRY_RUN=False - engaging torque on both arms")
        _force_torque_on(LEFT_PORT)
        _force_torque_on(RIGHT_PORT)

    print("[client] POST /reset")
    r = requests.post(f"{POLICY_URL}/reset", json={}, timeout=10)
    print(f"[client] reset response: {r.status_code} {r.text.strip()}")

    period = 1.0 / FPS
    max_step_rad = np.radians(MAX_DEG_PER_SEC) * period

    cmd_rad = None
    n = 0
    last_action = None
    try:
        while not interrupted and n < N_STEPS:
            tick0 = time.time()

            l_state = read_arm_state(left_ph, left_pkt, cal["left"])
            r_state = read_arm_state(right_ph, right_pkt, cal["right"])
            state12 = np.concatenate([l_state, r_state])

            top = grab_realsense_rgb(rs_pipe)
            lf = grab_v4l2_rgb(left_cam)
            rf = grab_v4l2_rgb(right_cam)

            obs = {
                "observation.state": state12.tolist(),
                "observation.images.top_rgb": encode_image(top),
                "observation.images.left_rgb": encode_image(lf),
                "observation.images.right_rgb": encode_image(rf),
            }
            t_pre = time.time()
            try:
                r = requests.post(f"{POLICY_URL}/infer", json=obs, timeout=15)
                action = np.array(r.json()["actions"][0], dtype=np.float32)
            except Exception as e:
                print(f"[client] infer error: {e}")
                continue
            t_infer = time.time() - t_pre

            jump = 0.0 if last_action is None else float(np.abs(action - last_action).max())
            last_action = action

            if cmd_rad is None:
                cmd_rad = state12.copy()
            delta = action - cmd_rad
            delta = np.clip(delta, -max_step_rad, max_step_rad)
            cmd_rad = cmd_rad + delta

            print(
                f"step {n:>3d}  infer={t_infer*1000:5.1f}ms  "
                f"L_obs={state12[1]:+5.2f}->tgt={action[1]:+5.2f} (lift)  "
                f"L_obs={state12[3]:+5.2f}->tgt={action[3]:+5.2f} (wflex)  "
                f"R_obs={state12[7]:+5.2f}->tgt={action[7]:+5.2f} (lift)  "
                f"jump={jump:.3f}"
            )

            if not DRY_RUN:
                try:
                    write_arm_action(left_ph, left_pkt, cal["left"], cmd_rad[:6])
                    write_arm_action(right_ph, right_pkt, cal["right"], cmd_rad[6:])
                except Exception as e:
                    print(f"[client] write_arm_action error: {e}")

            n += 1
            elapsed = time.time() - tick0
            if elapsed < period:
                time.sleep(period - elapsed)
    finally:
        print("[client] stopping")
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


if __name__ == "__main__":
    main()

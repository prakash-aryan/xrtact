# Architecture (Draft)

High-level component layout for the trimmed-scope MVP.

## Component diagram

```
                 ┌────────────────────────┐
                 │   Meta Quest 3         │
                 │   - WebXR client       │
                 │   - immersive-ar       │
                 │   - hand & controller  │
                 │     pose tracking      │
                 │   - MR passthrough     │
                 │     overlays           │
                 └─────┬────────────▲─────┘
                       │            │
                  HTTPS:8443   WSS:8442
                       │            │
                       ▼            │
                 ┌────────────────────────┐
                 │  WebXR-server          │  Container
                 │  (aiohttp + WebSocket) │
                 │  /web-ui statics       │
                 │  /vr WebSocket         │
                 └─────┬───────────▲──────┘
                       │           │
                       ▼           │
              ControlGoal queue    │ camera/state/action stream
                       │           │
                       ▼           │
                 ┌────────────────────────┐
                 │  control-loop          │  Container (motor)
                 │  - PyBullet IK (2 arms)│
                 │  - safety clamp        │
                 │  - lerobot.SOFollower  │
                 │     × 2                │
                 └─┬──────────┬───────────┘
                   │          │
                   ▼          ▼
                ttyACM0   ttyACM1     ← USB-serial to physical SO-ARM101 followers
                                            (host devices /dev/* mapped into container)

                 ┌────────────────────────┐
                 │  camera-daemon         │  Container
                 │  - pyrealsense2 (top)  │
                 │  - cv2 webcams (L,R)   │
                 │  - shared timestamp    │
                 │  - publish frames over │
                 │    Unix socket / pipe  │
                 └─┬──────────────────────┘
                   │
                   ▼
                 ┌────────────────────────┐
                 │  recorder              │  Container
                 │  - subscribe to state  │
                 │    + action + frames   │
                 │  - LeRobotDataset      │
                 │    writer (v0.4.x)     │
                 │  - one episode per     │
                 │    teleop session      │
                 └────────────────────────┘

                 ┌────────────────────────┐
                 │  validator (CI)        │  Container
                 │  - schema check        │
                 │  - 1-epoch ACT sanity  │
                 │    train               │
                 └────────────────────────┘
```

## Process boundaries

| service | language | responsibilities | exposed |
|---|---|---|---|
| **webxr-server** | Python (aiohttp) | serve HTTPS + WSS; relay WebXR pose frames to the control loop; relay camera frames + recording state back to client | `:8443/`, `:8442/ws` |
| **control-loop** | Python | consume `ControlGoal` queue, run PyBullet IK for both arms, clamp joint limits, write motor targets to `lerobot.SOFollower` × 2 | publishes `state`, `action`, `recording_flag` on internal pubsub |
| **camera-daemon** | Python | open RealSense + 2× webcams, publish synchronized frames at 30 fps with monotonic timestamps | publishes `images.{top_rgb, top_depth, left_rgb, right_rgb}` on internal pubsub |
| **recorder** | Python | subscribe to all control-loop and camera streams, write `LeRobotDataset` chunks when `recording_flag=true` | mounts `./datasets/` volume |
| **validator** | Python | one-shot job triggered by recorder's "episode complete" hook; schema-check + 1-epoch ACT smoke-train | exits 0 on pass |

Internal pubsub can be ZeroMQ, Redis pub/sub, or just unix-domain sockets. **Pick the simplest that handles 30 Hz on the local box.**

## Data formats

### LeRobotDataset (v0.4.x)

```
datasets/<dataset_name>/
├── meta/
│   ├── info.json           # global metadata, fps=30, total_episodes, total_frames
│   ├── episodes.parquet    # per-episode timestamps + index ranges
│   ├── stats.json
│   └── tasks.parquet       # task descriptions
├── data/
│   └── chunk-000/
│       └── episode_000000.parquet
└── videos/
    └── observation.images.top_rgb/
        └── chunk-000/
            └── file-000.mp4
```

Features each episode contains:

| key | dtype | shape | source |
|---|---|---|---|
| `observation.state` | float32 | (12,) | `[left_arm_6joints, right_arm_6joints]` raw motor positions in degrees |
| `action` | float32 | (12,) | target joint positions sent to followers, same layout |
| `observation.images.top_rgb` | uint8 | (H, W, 3) at 30 fps | RealSense color stream |
| `observation.images.left_rgb` | uint8 | (H, W, 3) at 30 fps | left wrist webcam |
| `observation.images.right_rgb` | uint8 | (H, W, 3) at 30 fps | right wrist webcam |
| `observation.top_depth` | uint16 (mm) | (H, W) at 30 fps | RealSense depth stream (should-have, not must-have) |
| `timestamp` | float64 | scalar | monotonic seconds since episode start |

Resolution proposal: `480 × 640` to match the LeHome spec we already verified ACT trains on. Easy to change later.

### WebSocket message format (telegrip-derived)

Existing telegrip pose message:

```json
{
  "type": "controller_pose",
  "arm": "right",
  "position": [x, y, z],          // VR-frame meters
  "wrist_roll_deg": 12.4,
  "trigger_pressed": true,
  "grip_pressed": true
}
```

For the MR project we add:

```json
{
  "type": "session_control",
  "action": "start_recording" | "stop_recording" | "calibrate_overlay"
}
```

Server → client (camera feedback at lower rate, e.g. 10 fps):

```json
{
  "type": "camera_frame",
  "view": "top" | "left" | "right",
  "format": "jpeg",
  "data_b64": "..."
}
```

## Coordinate frames

The hardest non-glamorous part of this project. Three frames involved:

| frame | origin | axes |
|---|---|---|
| **VR** | Quest 3 floor anchor at session start | X=right, Y=up, Z=back (toward operator) |
| **Robot world** | Table center between the two arm bases | X=forward (away from operator), Y=left, Z=up |
| **Each arm's base** | mount point of that follower | URDF defines |

Telegrip already does VR → Robot-world transformation in `core/kinematics.py`. The MR overlay needs the **inverse**: given a pose in robot-world (from FK of the real arms or target IK), render it in the VR scene at the right virtual position so it overlays the real hardware.

**Calibration approach:** at session start, the operator stares at a printed fiducial (e.g. ArUco marker) on the table center. WebXR captures the headset's pose at that moment as the "anchor" in VR-frame and we declare it equal to robot-world origin. Re-calibration available via a button.

## Hand-off summary

The student takes this folder, replaces the `~/telegrip` reference clone with their own working fork, and:

1. Gets dual-arm telegrip running with two followers wired to USB.
2. Adds the recorder service (LeRobotDataset writer) — the core pipeline contribution.
3. Replaces `web-ui/vr_app.js` with an `immersive-ar` WebXR app and adds the MR overlay logic (the scientific contribution).
4. Wraps each service in a Dockerfile and writes `docker-compose.yml` (the DevOps contribution).
5. Optionally integrates leader-arm benchmarking and CI for the thesis comparison study.

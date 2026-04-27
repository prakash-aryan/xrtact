# Session Log — 2026-04-25

Initial planning conversation. Took place in `~/lehome-challenge` (the user's parallel competition project) but pivoted to scoping this MR teleop project.

## What was reviewed

- `~/telegrip` — local clone of github.com/DipFlip/telegrip. Read the README + `SETUP.md` + `config.yaml` and walked the package layout (`telegrip/main.py`, `telegrip/control_loop.py`, `telegrip/core/{robot_interface, kinematics, visualizer}.py`, `telegrip/inputs/{vr_ws_server, web_keyboard}.py`, `web-ui/{index.html, vr_app.js}`).
- `~/Downloads/Project_5_MR_Teleoperation_SO101.pdf` — student project handout.

## Key understanding established

1. **Telegrip already covers a lot:** Quest WebXR client, hand-tracking → WebSocket → Python control loop, PyBullet IK/FK, lerobot SOFollower bridge for motor control, auto-generated SSL certs, single-arm and (in code) dual-arm modes.
2. **What telegrip is missing for the project:**
   - MR passthrough overlays (target poses, trajectory previews, joint limits, recording status drawn on top of the real arms)
   - LeRobotDataset writer / demo recording
   - Containerized pipeline (Docker Compose for motor + camera + recorder + validator)
   - Comparison/benchmark vs leader-arm teleop
3. **Telegrip does NOT use ROS2 / rosbridge.** It uses aiohttp WebSocket directly. The PDF specified rosbridge, but the supervisor decided to drop that requirement.

## Decisions taken in this session

| topic | decision | reason |
|---|---|---|
| ROS2 / rosbridge | **drop** | Telegrip already proves you don't need ROS2 to ship a working WebXR-to-SO-101 pipeline; saves student weeks of plumbing |
| Number of arms | **keep both (bimanual)** | Laundry / cloth folding is inherently bimanual — one gripper anchors, the other manipulates. ALOHA, Open-TeleVision, and most cloth-manipulation literature confirm this |
| Cameras | 2 wrist webcams + 1 overhead RealSense (RGB + depth) | Matches the typical `top_rgb / left_rgb / right_rgb` LeRobot triple, and gives a free `top_depth` channel from the same sensor |
| Tasks | pick-and-place + stacking remain as benchmarks; **laundry/folding is the long-term application** | Easy benchmarks for ACT comparison; folding is the eventual research story |
| MR passthrough overlays | **kept — this is THE differentiator** | Nobody has applied Open-TeleVision-style headset feedback to the SO-101 ecosystem yet |
| Docker Compose pipeline | **kept** | The "DevOps for Cyber-Physical Systems" course angle still needs containerization/CI |
| Benchmark vs leader-arm | **conditional — kept if 2 leader arms are available** | Otherwise drop and ship just the recording pipeline |

## What was NOT decided yet

- Whether the WebXR client extends `telegrip/web-ui/vr_app.js` directly or starts a new SimNav-XR-derived WebXR app.
- Concrete docker-compose service layout (one container per concern vs single container).
- LeRobot version to target (0.4.3 was used in lehome-challenge; latest may have moved on).
- Whether to use the Quest 3 hand-tracking path or controller path as the primary input (controller is more reliable; hand-tracking is more ergonomic).
- Calibration of MR overlays — how to register the virtual coordinate frame to the real-arm pose.
- Whether RealSense streams as `pyrealsense2` Python library or via a v4l2 fallback.

## Side context (LeHome challenge work, 2026-04-19 → 2026-04-25)

This project is **not** the LeHome challenge — but the same supervisor was concurrently working on a sim-only ICRA 2026 garment-folding submission in `~/lehome-challenge/`. Useful learnings that may transfer:

- **`LeRobotDataset` format:** chunks of parquet + chunked MP4 video files per camera key. Confirmed working with LeRobot 0.4.3 via `LeRobotDatasetMetadata` API (`m.episodes[ep]['dataset_from_index']` etc).
- **ACT recipe that works on bimanual SO-ARM:** ResNet-18 backbone, `chunk_size=100`, `temporal_ensemble_coeff=0.01`, `n_action_steps=1`, batch size 16, 120 k steps. Got 56.25 % overall on the LeHome benchmark.
- **Mirror augmentation matters:** L/R-flipping wrist cameras + swapping L/R arm joint blocks + sign-flipping `shoulder_pan` and `wrist_roll` (the joints whose rotation axis reverses under sagittal mirror) added ~25 pp to ACT scores.
- **Diffusion Policy needs effective batch ≥ 64** to train stably. bs=8 trains to 0% on top_long; bs=64 (DDP across 2 GPUs) trains to 67%.
- **Per-category specialists** beat joint models on hard categories (pant_long: 25% joint → 50% specialist).

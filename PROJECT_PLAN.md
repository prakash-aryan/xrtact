# Project Plan — Trimmed Scope

The full PDF goal is broad. This document defines a feasible MVP for a single student over one semester, plus stretch goals.

## One-line goal

Build an MR (mixed-reality) teleoperation system for **two SO-ARM101** robots that lets an operator wearing a **Meta Quest 3** record demonstrations directly into **LeRobotDataset** format, deployed as a **Docker Compose** stack, and validated by training an **ACT** policy on the recorded demos.

## Scope

### Must have (MVP — ships even if everything else is cut)

- [ ] Bimanual SO-ARM101 control over WebSocket from a WebXR client (single-arm telegrip, extended to two arms).
- [ ] Quest 3 controller-pose → bimanual end-effector targets via shared IK (PyBullet, two robots loaded into one sim scene).
- [ ] Live camera feeds back to the headset (at minimum: 1 overhead RealSense color stream as a heads-up panel).
- [ ] LeRobotDataset recording: a "start/stop" button on the WebXR UI writes one episode per session containing `observation.state`, `observation.images.{top_rgb, left_rgb, right_rgb}`, and `action`.
- [ ] Docker Compose `up` brings up the full stack on the host machine.
- [ ] ACT training succeeds on a small batch (10–20 demos) of recorded data and outputs a checkpoint.

### Should have (clear contribution beyond the MVP)

- [ ] **MR passthrough overlays** — virtual ghost arms showing target pose + recording status overlaid on the real arms via Quest 3 passthrough. This is the project's actual scientific differentiator.
- [ ] RealSense **depth** stream piped into `observation.top_depth`.
- [ ] Hand-tracking input as an alternative to controllers.
- [ ] Demo replay mode — load a recorded LeRobotDataset episode and watch the virtual ghost arms re-execute it in MR.

### Nice to have (only if calendar allows)

- [ ] Quantitative benchmark vs a physical leader-arm setup (success rate, smoothness, demos/hour, trained ACT policy success rate). Requires also having two leader arms wired up.
- [ ] CI pipeline: a job that runs on every dataset push, validates schema, runs a 1-epoch ACT sanity check.
- [ ] HuggingFace Hub publication of a public `LeRobotDataset` of all recorded demos.

### Explicitly out of scope

- ROS2 / rosbridge_suite (decision in SESSION_LOG.md). The aiohttp WebSocket bus already used by telegrip is sufficient.
- Generic multi-robot manipulator support — SO-ARM101 only.
- Long-horizon imitation tasks beyond pick-and-place / stacking / one cloth-folding action. Full laundry sequences are a research project of their own.

## What survives from telegrip vs gets rewritten

| telegrip module | reuse / rewrite / replace |
|---|---|
| `telegrip/inputs/vr_ws_server.py` | **Reuse** — the WebSocket plumbing for hand pose data is exactly what we need. |
| `telegrip/inputs/web_keyboard.py` | **Reuse** as fallback control. Useful for testing without the headset. |
| `telegrip/core/kinematics.py` | **Extend** — currently single-arm IK. Need two PyBullet-loaded SO-ARM101s in one scene with their own IK chains. |
| `telegrip/core/robot_interface.py` | **Extend** — already has the `lerobot.SOFollower` bridge and dual-arm path (see `left_arm.enabled` / `right_arm.enabled` in config). Need to wire both arms simultaneously. |
| `telegrip/control_loop.py` | **Extend** — same control loop with two follower channels. |
| `telegrip/web-ui/vr_app.js` | **Replace** for the MR overlay version — current Three.js scene assumes pure-VR, not passthrough. Need a new WebXR session with `immersive-ar` and `local-floor` reference space + pose anchoring. |
| `telegrip/web-ui/{index.html, interface.js, styles.css}` | **Reuse** as desktop debug UI. |

## What does NOT exist yet and must be built from scratch

- **LeRobotDataset writer** — a Python module that subscribes to `(state, action, images)` on the same control loop tick and writes parquet + MP4 chunks in the LeRobot v0.4.x layout. Reference implementation lives in `lerobot/scripts/lerobot_record.py`.
- **MR overlay logic** — virtual ghost-arm meshes loaded from the SO-ARM101 URDF, rendered into the WebXR scene at the same world-space position as the real arms. Needs a registration / calibration step (e.g. user looks at a fiducial on the table while wearing the headset).
- **Camera capture container** — an OpenCV / pyrealsense2 daemon that pushes frames over a shared volume or an internal WebSocket so the dataset writer can grab synchronised frames.
- **Docker Compose orchestration** — one service per concern: `motor`, `camera`, `webxr-server`, `recorder`, `validator`.

## Milestones (rough)

| month | goal |
|---|---|
| 1 | Telegrip running on lab machine with both SO-ARM101 arms wired, dual-arm IK working, recording stub that prints joint states to stdout. |
| 2 | LeRobotDataset writer producing valid `LeRobotDataset` chunks. ACT trains on dummy data without errors. |
| 3 | MR overlay v0 — passthrough WebXR session shows a static virtual cube on the table where the user gazed. Camera feeds appear as heads-up panels. |
| 4 | MR overlay v1 — virtual ghost arms tracking target poses. Real recording session collects 20+ demos of a pick-and-place task. |
| 5 | Docker Compose stack. ACT policy trained on collected demos shows non-trivial success on the held-out task. |
| 6 | Stretch: leader-arm comparison run, CI pipeline, paper draft. |

## Risk register

| risk | likelihood | impact | mitigation |
|---|---|---|---|
| WebXR latency too high for 30 Hz control | medium | high | Telegrip already runs at this rate single-arm; benchmark early. Fallback: control rate 15 Hz, interpolate on the robot side. |
| MR overlay registration drift | high | medium | Use a fiducial on the table for an initial pose-anchor reset; allow re-calibration via a button. |
| RealSense + 2× webcams oversaturate USB bus | medium | medium | RealSense on USB3, webcams on separate USB controllers if possible. |
| LeRobot dataset format moves between versions | medium | low | Pin LeRobot version in `pyproject.toml`. The format we used in `lehome-challenge` (LeRobot 0.4.3) is documented. |
| Both arms reach for the same workspace point and self-collide | medium | high | Joint limit clamping + simple workspace-overlap check in the IK module. |
| Quest 3 hand tracking too jittery for fine manipulation | low (controllers used by default) | medium | Default to controllers; hand tracking is should-have, not must-have. |

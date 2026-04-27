# Next Steps for a Fresh Claude Session

Drop into this folder, read `README.md` then `PROJECT_PLAN.md`, then pick a track below.

## Track A: project planning / writing (no code)

Useful if the next session is more about scoping or producing a student-facing handout.

- [ ] Refine `PROJECT_PLAN.md` milestones into week-by-week deliverables.
- [ ] Write a 1-page student handout (different from the formal PDF) summarising the trimmed scope, deliverables, success criteria, and suggested order of attack.
- [ ] Draft acceptance criteria per deliverable (what does "done" look like?).
- [ ] List candidate evaluation tasks for the leader-arm comparison study (pick-and-place variants, stacking variants, simple cloth fold).
- [ ] Survey the related work cited in the PDF (Open-TeleVision, unity_ros_teleoperation, VR Dual-Arm Teleoperation MDPI 2026) and write a half-page positioning note that says exactly what this project does that those don't.

## Track B: bring up telegrip on the lab machine (hands-on)

Useful if hardware is in the room and the goal is "make the robots move from the headset".

Order matters here — each step builds on the last.

1. [ ] **Find both USB serial ports.** `lerobot-find-port` once with each arm plugged in alone. Update `~/telegrip/config.yaml` with the actual ports.
2. [ ] **Calibrate each follower.** `lerobot-calibrate --robot.type=so101_follower --robot.port=... --robot.id=left_follower` and `right_follower`. Calibration files land in `~/.cache/huggingface/lerobot/calibration/robots/so_follower/`.
3. [ ] **Edit `telegrip/core/robot_interface.py`** at the marked lines (~111, ~119) to make sure `left_config.id` / `right_config.id` match the calibration ids you just used.
4. [ ] **Set both arms to enabled** in `~/telegrip/config.yaml` — `left_arm.enabled: true` and `right_arm.enabled: true`.
5. [ ] **Run telegrip headless once with no robot** to confirm the WebXR server boots:  `python -m telegrip --no-robot --log-level info`. Open `https://<laptop-ip>:8443` in a normal browser, accept the cert, see the desktop UI.
6. [ ] **Connect the headset.** Same URL in the Quest browser. Verify hand pose data flows in (telegrip will log `controller_pose` messages).
7. [ ] **Enable robots.** Drop `--no-robot`, see both arms wake up, hold the grip button on a controller, watch the corresponding arm follow.
8. [ ] **Run baseline data flow test:** with both arms tracking, hand-write a 50-line Python script that subscribes to telegrip's internal command queue and prints `(left_state, right_state, action, timestamp)` to stdout at 30 Hz. This is the precursor to the recorder.

## Track C: extend telegrip toward the recorder service

Once Track B works, this is the highest-value first feature: writing demos to disk in `LeRobotDataset` format.

1. [ ] Add a third input event to the WebXR client: a "Record" toggle button on the desktop debug UI (start with the easy case, then port to MR later).
2. [ ] In `telegrip/main.py`, hook `start/stop_recording` events.
3. [ ] Write `telegrip/recorder.py` that accumulates `(state, action, top_rgb, left_rgb, right_rgb, timestamp)` rows during the active period and writes one `episode_NNN.parquet` + matching MP4s on stop, following `lerobot.scripts.lerobot_record` as a reference.
4. [ ] Verify the resulting dataset loads cleanly with `LeRobotDatasetMetadata("local", root="./datasets/test")` and that `len(meta.episodes) == 1` after one recording.
5. [ ] Train a 1-epoch ACT smoke job on the recorded episode just to confirm the format is valid:
   ```bash
   lerobot-train policy=act dataset_repo_id=local dataset_root=./datasets/test
   ```
6. [ ] Once that works on a single episode, batch-record 20 demos of pick-and-place and run ACT for real.

## Track D: MR passthrough overlay (the actual research contribution)

Only after Tracks B + C work end-to-end. This is the publishable bit.

1. [ ] Replace the WebXR session in `telegrip/web-ui/vr_app.js` with `xr.requestSession('immersive-ar', { requiredFeatures: ['local-floor', 'hit-test'], optionalFeatures: ['hand-tracking'] })`.
2. [ ] Disable the existing Three.js virtual environment background — passthrough mode shows the real world.
3. [ ] Load the SO-ARM101 URDF (already at `~/telegrip/URDF/SO100/urdf/so100.urdf` — likely close enough, or pull a real SO-101 URDF) into the WebXR scene as a translucent ghost mesh.
4. [ ] Implement the calibration step from `ARCHITECTURE.md` — operator stares at a fiducial, button press anchors the virtual robot-world frame to that headset pose.
5. [ ] Render two ghost arms at the FK pose of the IK solution (the **target** the IK is trying to reach) so the operator sees where the arm is *about to* go.
6. [ ] Add a recording-status indicator floating in MR — solid red dot when actively recording.
7. [ ] User-study a few tasks comparing recording quality with overlays vs without. Quantify: trajectory smoothness, completion time, demos-per-hour.

## Track E: containerization

Only after Tracks B + C are stable. Container churn before the core works is wasted effort.

1. [ ] Write `Dockerfile` for the webxr-server / control-loop / camera-daemon / recorder / validator services.
2. [ ] Write `docker-compose.yml` orchestrating them. Mount `/dev/ttyACM*` and `/dev/video*` from host into the appropriate containers. Use `network_mode: host` for the webxr-server (Quest needs to reach 8443 directly).
3. [ ] Test that `docker compose up` starts the full stack and the headset can still connect.
4. [ ] Add a CI job (GitHub Actions or local `act`) that runs the validator container against a tiny fixture dataset on every push.

## Tips for the next Claude

- Telegrip is **open source upstream**. If you change it locally, fork it on GitHub before making non-trivial edits — easier to share with the student and merge upstream later.
- LeRobot version drift is real. Pin to whatever version the student starts with (likely latest, not `0.4.3` like LeHome used). The dataset format has occasionally moved across minor versions.
- Quest 3 needs HTTPS to allow WebXR. Telegrip auto-generates self-signed certs; if pushing to production-ish, get a `mkcert`-issued local CA cert so the headset doesn't keep showing the warning page.
- For dual-arm testing without two arms, run telegrip with `--no-robot` and watch PyBullet visualization. Both arms render even if neither is plugged in.
- The lehome-challenge `LeRobotDataset` path is a working reference — `~/lehome-challenge/Datasets/example/four_types_merged/`. Useful for sanity checks.

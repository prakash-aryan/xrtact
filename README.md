# xrtact

XR teleoperation + imitation learning for a bimanual SO-ARM101 setup.

Quest 3 controllers drive both arms through [telegrip](https://github.com/DipFlip/telegrip)'s WebXR + IK pipeline. A side process owns the SO-101 motors via raw `scservo_sdk`, captures three cameras (overhead RealSense + two wrist Logitechs), and writes a `LeRobotDataset` whose schema matches LeHome / [LeIsaac](https://github.com/LightwheelAI/leisaac) so demos drop straight into fine-tuning the LeHome ICRA 2026 garment-folding policy.

<img width="1119" height="592" alt="act_policy" src="https://github.com/user-attachments/assets/fb159862-c652-44e0-97c4-ee8ff8205379" />


## Layout

| path | what |
|---|---|
| `scripts/home.py` | drive both arms to a saved home pose (raw scservo, robust) |
| `scripts/record_bimanual.py` | record a bimanual VR demo to `LeRobotDataset` |
| `scripts/teleop_real_arm.py` | drive a single arm from VR with no recording (debug) |
| `scripts/realbot_client.py` | run the LeHome policy on the real bots (HTTP `/infer`) |
| `scripts/home_pose.example.json` | example shape - copy to `home_pose.json` and write your own raw counts |
| `scripts/zmq_sim_smoketest.py` | synthetic publisher for testing the bridge without VR |
| `scripts/home_arms.py` | older lerobot-based homer (kept; superseded by `home.py`) |
| `src/xrtact/` | Python package skeleton for in-tree code |
| `vendor/telegrip/` | telegrip submodule (WebXR + IK + ZMQ publisher, patched here) |
| `vendor/leisaac/` | LeIsaac submodule (Isaac Lab tasks, robot configs, conversions) |

`vendor/telegrip` carries our patches for: a `record` message type that toggles a recording flag (X+A controller combo), an in-VR HUD that renders that flag, and a `recording` field included in every `sim_bridge` ZMQ message.

## Configure

One-time per machine:

1. **Clone with submodules**
   ```bash
   git clone --recurse-submodules https://github.com/prakash-aryan/xrtact.git
   cd xrtact
   ```

2. **Calibrate both SO-101 follower arms**
   ```bash
   uv pip install -e ./vendor/telegrip   # provides lerobot CLI
   lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM0 --robot.id=left_follower
   lerobot-calibrate --robot.type=so101_follower --robot.port=/dev/ttyACM1 --robot.id=right_follower
   ```
   Saves to `~/.cache/huggingface/lerobot/calibration/robots/so_follower/{left,right}_follower.json`. The recorder + home script both read these.

3. **Cameras**
   - One Intel RealSense D435 (overhead). Find its serial with `rs-enumerate-devices | grep Serial` and set it via `XRTACT_REALSENSE_SERIAL` (the recorder errors out at startup if unset).
   - Two Logitech C270s (one per wrist). They share an identical USB serial (`200901010001`) which means they MUST be on different USB host controllers (different physical ports, not just different ports on the same hub). Verify with `lsusb -t`.

4. **Set the env vars for your machine** (put in `~/.bashrc` or a `.envrc`):
   ```bash
   export XRTACT_LEFT_PORT=/dev/ttyACM0    # left arm serial port
   export XRTACT_RIGHT_PORT=/dev/ttyACM1   # right arm serial port
   export XRTACT_LEFT_CAM_INDEX=10         # left wrist  /dev/videoN
   export XRTACT_RIGHT_CAM_INDEX=8         # right wrist /dev/videoN
   export XRTACT_REALSENSE_SERIAL=         # paste your RealSense serial here
   ```
   Defaults for ports + cam indices match a typical setup; the RealSense serial has no default and the script will tell you so.

5. **Save your home pose**

   Pose both arms by hand where you want "home" to be (release torque first if motors are locked), then snapshot to `scripts/home_pose.json` (gitignored, per-machine):
   ```bash
   python -c "
   import json
   from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS
   out = {'left': {}, 'right': {}}
   for s, p in [('left', '/dev/ttyACM0'), ('right', '/dev/ttyACM1')]:
       ph = PortHandler(p); ph.openPort(); ph.setBaudRate(1_000_000)
       pkt = PacketHandler(0)
       for i in range(1, 7):
           v, _, _ = pkt.read2ByteTxRx(ph, i, 56)
           out[s][str(i)] = int(v)
       ph.closePort()
   json.dump(out, open('scripts/home_pose.json', 'w'), indent=2)
   "
   ```
   `scripts/home_pose.example.json` shows the shape if you'd rather hand-edit a copy.

6. **(Optional) Cloudflared named tunnel - only if you can't use LAN/WiFi**

   The intended path is plain LAN: have the Quest and the laptop on the same WiFi, then open `https://<laptop-ip>:8443` in the headset. Use cloudflared only as a backup, e.g. when your WiFi has AP-isolation (iPhone hotspots) or when working off-site.

   Setup (one-time):
   ```bash
   cloudflared tunnel login                       # browser login, picks your zone
   cloudflared tunnel create xrtact-teleop        # creates a UUID + credentials json
   ```
   Then write `~/.cloudflared/config.yml` with two ingress rules - one for the web UI, one for the WebSocket. **Do not check this file in.** Use the placeholders below; replace with your domain:
   ```yaml
   tunnel: <your-tunnel-uuid>
   credentials-file: /home/<user>/.cloudflared/<your-tunnel-uuid>.json
   ingress:
     - hostname: teleop.<your-domain>
       service: https://localhost:8443
       originRequest:
         noTLSVerify: true        # telegrip serves a self-signed cert
     - hostname: ws.<your-domain>
       service: https://localhost:8442
       originRequest:
         noTLSVerify: true
     - service: http_status:404
   ```
   Add the DNS records and run the tunnel:
   ```bash
   cloudflared tunnel route dns xrtact-teleop teleop.<your-domain>
   cloudflared tunnel route dns xrtact-teleop ws.<your-domain>
   cloudflared tunnel --config ~/.cloudflared/config.yml run xrtact-teleop
   ```
   The WebXR JS auto-derives the WS hostname from the page hostname (`teleop.<X>` → `wss://ws.<X>`), so no JS edits are needed.

## Run

### Home both arms

```bash
python scripts/home.py                  # both, ~4s ramp
python scripts/home.py --left-only --duration 5
```
Idempotent and safe from any starting state - reads current pose, ramps Goal_Position to the saved home over `--duration` seconds.

### Record a bimanual VR demo

Two terminals (three if you also need cloudflared):

```bash
# 1. publish joint targets from VR
cd vendor/telegrip && .venv/bin/python -u -m telegrip --no-robot --log-level info

# 2. drive arms + record dataset
python -u scripts/record_bimanual.py \
    --repo-id local/realarm_bimanual_foldshirt_v1 \
    --task "Fold the full-sleeve shirt." \
    --max-episode-s 600

# 3. (optional, only if Quest can't reach the laptop directly)
cloudflared tunnel --config ~/.cloudflared/config.yml run xrtact-teleop
```

In the headset:
1. Open `https://<laptop-ip>:8443/` (LAN) or `https://teleop.<your-domain>/` (cloudflared), tap **enter xr**, accept the self-signed cert if prompted.
2. Press **X (left controller) + A (right controller) together** to toggle recording. The HUD shows `● REC` red when capturing, `○ REC` grey when paused. Per-arm grip + controller motion drives the corresponding arm.
3. Press X+A again to pause; Ctrl+C in the recorder terminal to end the episode + finalise the dataset (wait for `[record] dataset finalized` - do not SIGKILL or the parquet ends up corrupt).

Output: `~/.cache/huggingface/lerobot/<repo_id>/` - 12-dim state + 12-dim action in URDF radians, three 640×480 RGB videos.

### Single-arm teleop without recording (debug)

```bash
cd vendor/telegrip && .venv/bin/python -u -m telegrip --no-robot &
python scripts/teleop_real_arm.py --arm right
```

### Run the LeHome pretrained policy on the real arms

```bash
docker run --rm -d --gpus all -p 8080:8080 --name lehome_realbot merabro/lehome-r77:v5-gpu
python scripts/realbot_client.py    # set DRY_RUN=False inside to actually move motors
```


### Run recording pipeline
```bash
python scripts/make_demo.py \
    --repo-id local/realarm_bimanual_foldshirt_v1 \
    --task "Fold the full-sleeve shirt." \
    --max-episode-s 600
```

## Known issues

- **Coordinate-frame mismatch (telegrip ↔ real arms).** `telegrip` runs IK in PyBullet (URDF frame) but writes via `lerobot.SOFollower` which interprets values in the lerobot-calibration frame. The two are usually NOT the same physical position. The recorder works around this with **anchor mode**: when you press X+A it captures `(arm_pose, zmq_target)` and from then on commands `arm = arm_anchor + (zmq_target - zmq_anchor)`, so the arm tracks the *delta* of your hand motion rather than the absolute IK output. PyBullet visualization will still look offset from the real arm - that is the same root cause.
- **Per-joint sign flips.** Whether `range_min motor count` corresponds to URDF positive or negative depends on motor wiring. Toggle `INVERT[side][joint]` between `+1` and `-1` in `record_bimanual.py` per joint if you observe inverted control on a specific axis.
- **Two C270 webcams on the same USB controller fail.** Both share USB serial `200901010001`; the kernel UVC driver runs only one of them when they're on the same host controller. Plug them into ports on different USB controllers (verify with `lsusb -t`).
- **`lerobot SOFollower.connect()` flakiness.** Strict ping with `num_retry=0` aborts on a single dropped packet (random "missing motor id=N" errors). The recorder bypasses this by using raw `scservo_sdk` directly; `realbot_client.py` monkey-patches `FeetechMotorsBus.ping` to retry.
- **`SOFollower.connect()` briefly drops torque** which lets gravity drop the arm. Mitigation: every script that opens the bus calls `_force_torque_on(port)` immediately after connect (Goal_Position = current, Torque_Enable = 1).
- **Motor 2 overheats in some home poses.** The shoulder_lift STS3215 (50% torque limit) trips its overheat protection if it has to fight gravity continuously. Save a low-stress home pose where shoulder_lift is below horizontal.
- **Killing the recorder mid-finalize corrupts the parquet.** `save_episode()` writes the final parquet footer + encodes MP4s in one shot at episode end. Always SIGINT (Ctrl+C) and wait for `[record] dataset finalized` before exiting; do not SIGKILL.
- **Quest browser caches `vr_app.js`.** When iterating on the WebXR JS, bump the `?v=...` query in `vendor/telegrip/web-ui/index.html` so the headset re-fetches.

## Done / left

Done:

- [x] WebXR teleop end-to-end through telegrip + cloudflared tunnel
- [x] `scripts/home.py` - robust raw-SDK homing, recovers from any starting state
- [x] `scripts/record_bimanual.py` - 12-dim URDF-rad state/action + 3 cameras, LeHome-compatible schema
- [x] X+A button combo + in-VR HUD for recording start/stop
- [x] Anchor-mode delta tracking so the recorder doesn't slam the arms on toggle
- [x] Per-joint sign correction (`INVERT` table) for motor/URDF direction mismatches
- [x] Gripper convention bridging (telegrip 0..45 ↔ lerobot RANGE_0_100)
- [x] LeHome policy server (`merabro/lehome-r77:v5-gpu`) brought up + dry-run client validated end-to-end
- [x] Velocity clamp + force-torque-on guards across all motor-driving scripts

To do (resolve open issues first; recording comes after):

Pre-recording fixes:

- [x] Mount RealSense at the sim camera height (~50-60 cm above table) so the policy actually sees the workspace, not just the cloth
- [x] Aim wrist cameras forward + slightly down so the gripper jaws sit at frame bottom (matching the sim wrist-cam pose)
- [X] Decouple camera capture from the control loop (background thread) to lift the recorder off the current ~10 Hz floor toward 30 Hz
- [x] Decouple recording from motor writes (currently the X+A flag gates both - separate them so the operator can teleop without recording every time)
- [X] Re-calibrate lerobot at the URDF zero pose (or patch telegrip's `SOFollower` boundary) so PyBullet visualisation matches the real arms and anchor mode is no longer needed
- [x] Confirm per-joint `INVERT` signs across all 12 joints empirically (currently 3 are guessed)
- [x] Add `make_demo` Python entrypoint that runs the whole record cycle (home → telegrip → tunnel → recorder) from one command
- [ ] Ship a small Web UI tile that visualises the recording flag + episode counter so the operator doesn't need to read terminal logs

Recording + training (after the above):

- [ ] Record **10 sample fold-shirt demos** end-to-end (smoke check the dataset format + workflow)
- [ ] Fine-tune the LeHome ACT `act_longer` checkpoint on those 10 demos and run it on the real bots - confirm the loop works (model loads, actions are sane, no schema mismatch) even if it doesn't fold yet
- [ ] Once the 10-demo loop is healthy, record ~50 high-quality bimanual fold-shirt demos
- [ ] Fine-tune again on the full 50 and evaluate on the real bots

## Acknowledgments

Built on [telegrip](https://github.com/DipFlip/telegrip) (MIT) and [LeIsaac](https://github.com/LightwheelAI/leisaac) (Apache 2.0). MIT-licensed.

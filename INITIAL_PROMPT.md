# Initial Prompt for a Fresh Claude Session

Copy one of the prompts below into the new Claude session. The first one is the general default — Claude reads the existing notes and waits for direction. The others are focused on a specific track.

---

## General default (recommended for the very first session)

```
You're picking up a project planning workspace. Before doing anything else, read:

1. README.md
2. SESSION_LOG.md (what was discussed in earlier sessions and what was decided)
3. PROJECT_PLAN.md (trimmed scope, MVP, milestones, risks)
4. ARCHITECTURE.md (component diagram, ports, dataset format, coordinate frames)
5. NEXT_STEPS.md (concrete tracks A through E)

After reading, summarise back to me in 5-10 lines:
- What this project is in one sentence
- The 3 most important decisions already taken
- What's currently the highest-ROI next step

Then ask me which track from NEXT_STEPS.md I want to work on. Don't start writing code or files until I tell you which track.
```

---

## Track A — Planning / writing focus (no hardware nearby)

```
Read README.md, SESSION_LOG.md, PROJECT_PLAN.md, ARCHITECTURE.md, NEXT_STEPS.md.

I want to work on Track A from NEXT_STEPS.md (project planning / writing).

Specifically, today let's:
1. Refine the milestones in PROJECT_PLAN.md from monthly into week-by-week deliverables.
2. Draft a 1-page student-facing handout based on the trimmed scope (different from the formal PDF in ~/Downloads/Project_5_MR_Teleoperation_SO101.pdf).
3. Write acceptance criteria for each must-have deliverable.

Save your output as new markdown files in this folder. Don't modify the existing notes — they're a session log.
```

---

## Track B — Hardware bring-up (lab session, hardware in front of you)

```
Read README.md, SESSION_LOG.md, NEXT_STEPS.md (especially Track B).

We have:
- 2× SO-ARM101 follower arms, assembled, not yet calibrated on this machine
- 2× USB webcams
- 1× Intel RealSense depth camera
- 1× Meta Quest 3S

The reference clone is at ~/telegrip/. Read its README.md and SETUP.md.

Walk me through Track B step-by-step. After each step, wait for me to confirm
the result before moving on. Don't run sudo commands without asking. Default to
running things in the foreground so I can watch output, not background.

Specifically I do NOT want to: install ROS2, fork telegrip on GitHub yet, or
write any new Python code today. We're getting telegrip working unmodified with
both arms first. Track C and D code changes come later.
```

---

## Track C — Recorder service (telegrip works with both arms; build the dataset writer)

```
Read README.md, SESSION_LOG.md, ARCHITECTURE.md (sections "LeRobotDataset" and
"Process boundaries"), NEXT_STEPS.md (Track C).

Telegrip is already running with both arms on this machine. I'm now ready to
add the recorder service: a module that subscribes to (state, action, images)
on the control loop tick and writes a LeRobotDataset chunk to disk when a
"Record" button is pressed in the WebXR UI.

Reference implementation: lerobot/scripts/lerobot_record.py in the cloned
lerobot tree at ~/telegrip/lerobot/.

For today's session, I want a working `telegrip/recorder.py` that:
1. Plugs into telegrip's existing main.py without breaking single-arm mode.
2. Triggered by a new WebSocket message `{type: session_control, action: start_recording|stop_recording}`.
3. Writes one episode per recording session, files in ./datasets/<dataset_name>/.
4. Uses the v0.4.x layout documented in ARCHITECTURE.md.
5. Has minimal dependencies — re-use what telegrip already imports.

Before writing code, read telegrip/main.py and telegrip/control_loop.py and tell me
exactly where the recorder hooks in. Then propose a 50-line skeleton and I'll
review it before you flesh it out.
```

---

## Track D — MR passthrough overlays (the actual research bit)

```
Read README.md, SESSION_LOG.md, ARCHITECTURE.md (especially "Coordinate frames"),
NEXT_STEPS.md (Track D).

Tracks B and C already work. Today I want to start Track D: replacing
telegrip/web-ui/vr_app.js with an MR passthrough version that overlays virtual
ghost SO-ARM101 arms on the real hardware via the Quest 3.

The hard sub-problems, in order:
1. Switch the WebXR session to immersive-ar with hand-tracking + local-floor.
2. Add a calibration step where the user looks at a fiducial on the table and
   presses a button to anchor the robot-world frame.
3. Load the SO-101 URDF as a translucent ghost mesh in Three.js.
4. Render two ghost arms at the FK pose of the current IK target so the
   operator sees the intended motion *before* the real arm executes.

For today's session, just nail step 1 and 2 — get a working immersive-ar
WebXR app that places a single virtual cube on the table at a calibrated
anchor point. Skip the URDF loading and ghost arms; those come next session.

Test plan: Quest 3 connects to https://<ip>:8443, enters immersive-ar, sees
a green cube floating where I tell it to place one, and the cube stays put
when I walk around the table.
```

---

## Track E — Containerization (everything else works; package it up)

```
Read README.md, ARCHITECTURE.md (especially "Component diagram" and
"Process boundaries"), NEXT_STEPS.md (Track E).

The full stack works on bare metal: telegrip with two arms, recorder writes
LeRobotDataset, MR overlays render on the headset. Time to wrap it in
docker-compose.

For today, just produce:
1. A Dockerfile for the webxr-server/control-loop service (most complex one).
2. A docker-compose.yml stub showing all five services listed in ARCHITECTURE.md
   with their volumes, /dev/* device mounts, and exposed ports.
3. A README section explaining `docker compose up` end-to-end.

Use Debian-slim base images. Pin exact versions. Don't worry about CI yet.
```

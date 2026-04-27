# xrtact

XR teleoperation of dual SO-ARM101 arms in Isaac Lab (LeIsaac), driven from a Quest 3 via [telegrip](https://github.com/DipFlip/telegrip). Bridges WebXR controllers to bimanual SO-101 articulations through a ZMQ JSON publisher; same pipeline retargets to real hardware later.

Maintainer: [Prakash Aryan](https://github.com/prakash_aryan) ([prakasharyan25@gmail.com](mailto:prakasharyan25@gmail.com))

## Run

```bash
# terminal 1
cd ~/telegrip && .venv/bin/python -u -m telegrip --no-robot

# terminal 2
cd ~/xrtact/vendor/leisaac
OMNI_KIT_ACCEPT_EULA=Yes ~/lehome-challenge/.venv/bin/python \
  -m scripts.environments.teleoperation.teleop_se3_agent \
  --teleop_device bi-so101webxr \
  --task LeIsaac-SO101-FoldCloth-BiArm-Direct-v0
```

Quest 3: open `https://<laptop-ip>:8443`, accept cert, hold a controller grip and move.

## Layout

- `vendor/telegrip/` - WebXR teleop server fork (with `sim_bridge.py`)
- `vendor/leisaac/` - LeIsaac fork (with `BiSO101WebXR` device)
- `src/xrtact/` - new project code
- `scripts/zmq_sim_smoketest.py` - synthetic publisher, verifies the bridge without telegrip/VR

## Acknowledgments

Built on [telegrip](https://github.com/DipFlip/telegrip) (MIT) and [LeIsaac](https://github.com/LightwheelAI/leisaac) (Apache 2.0). MIT-licensed.

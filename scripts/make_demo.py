from pathlib import Path
import argparse
import os
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent.parent
scripts = ROOT / "scripts"

parser = argparse.ArgumentParser()
parser.add_argument("--repo-id", required=True)
parser.add_argument("--task", required=True)
parser.add_argument("--max-episode-s", type=int, default=600)
args = parser.parse_args()

# --repo-id local/test_1.1 --task "Testing" --max-episode-s 600

def stop_process(proc: subprocess.Popen | None, name: str):
    if proc is None or proc.poll() is not None:
        return

    print(f"\nStopping {name}...")

    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print(f"{name} did not stop, terminating...")
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        print(f"{name} still alive, killing...")
        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


telegrip_proc = None
record_proc = None

try:
    subprocess.run(
        [sys.executable, str(scripts / "home.py")],
        check=True,
    )

    telegrip_proc = subprocess.Popen(
        ["telegrip", "--no-robot", "--log-level", "info"],
        start_new_session=True,
    )

    time.sleep(1)

    record_proc = subprocess.Popen(
        [
            sys.executable,
            str(scripts / "record_bimanual.py"),
            "--repo-id", args.repo_id,
            "--task", args.task,
            "--max-episode-s", str(args.max_episode_s),
        ],
        start_new_session=True,
    )

    record_proc.wait()

except KeyboardInterrupt:
    print("\nCtrl+C received.")

finally:
    stop_process(record_proc, "record_bimanual")
    stop_process(telegrip_proc, "telegrip")
    subprocess.run(
        [sys.executable, str(scripts / "reset_pose.py"), "--pose", f'{scripts}/zero_pose.json'],
        check=True,
    )
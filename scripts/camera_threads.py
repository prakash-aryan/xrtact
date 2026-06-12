import cv2
import numpy as np
import pyrealsense2 as rs

import threading
import time

# ---------------------------------------------------------------------------
# Background camera capture
# ---------------------------------------------------------------------------

WIDTH, HEIGHT = 640, 480
FPS = 30

class CameraThread(threading.Thread):
    """Grabs frames continuously into a single shared slot.

    The main loop calls .get() to read the latest frame without blocking.
    If no frame has arrived yet, get() returns None (caller should skip or
    retry). A threading.Event signals the thread to stop cleanly.
    """

    def __init__(self, name: str):
        super().__init__(name=name, daemon=True)
        self._frame: np.ndarray | None = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._ready_event = threading.Event()   # set once first frame arrives
        self.error: Exception | None = None

    def get(self) -> np.ndarray | None:
        """Return the most recent frame (RGB, HxWx3) or None if not yet available."""
        with self._lock:
            return None if self._frame is None else self._frame.copy()

    def wait_ready(self, timeout: float = 10.0) -> bool:
        """Block until the first frame has been captured (or timeout)."""
        return self._ready_event.wait(timeout)

    def stop(self) -> None:
        self._stop_event.set()

    def _put(self, frame: np.ndarray) -> None:
        with self._lock:
            self._frame = frame
        self._ready_event.set()


class RealSenseCameraThread(CameraThread):
    def __init__(self, serial: str):
        super().__init__(name="cam-realsense")
        self._serial = serial
        self._pipe: rs.pipeline | None = None

    def run(self) -> None:
        try:
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_device(self._serial)
            cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.rgb8, FPS)
            pipe.start(cfg)
            self._pipe = pipe
            # warm-up
            for _ in range(5):
                pipe.wait_for_frames(timeout_ms=2000)
            while not self._stop_event.is_set():
                frames = pipe.wait_for_frames(timeout_ms=2000)
                arr = np.asanyarray(frames.get_color_frame().get_data())
                if arr.shape[:2] != (HEIGHT, WIDTH):
                    arr = cv2.resize(arr, (WIDTH, HEIGHT))
                self._put(arr)
        except Exception as e:
            self.error = e
        finally:
            if self._pipe is not None:
                try:
                    self._pipe.stop()
                except Exception:
                    pass

    def stop(self) -> None:
        self._stop_event.set()


class V4L2CameraThread(CameraThread):
    def __init__(self, index: int):
        super().__init__(name=f"cam-v4l2-{index}")
        self._index = index
        self._cap: cv2.VideoCapture | None = None

    def run(self) -> None:
        try:
            cap = cv2.VideoCapture(self._index, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, FPS)
            if not cap.isOpened():
                raise RuntimeError(f"could not open /dev/video{self._index}")
            self._cap = cap
            time.sleep(0.3)  # let the camera settle before reading
            # warm-up: drain any buffered stale frames
            warmed = False
            for _ in range(10):
                ok, frame = cap.read()
                if ok and frame is not None:
                    warmed = True
                    break
            if not warmed:
                raise RuntimeError(f"/dev/video{self._index} produced no frames during warm-up")
            while not self._stop_event.is_set():
                ok, frame = cap.read()
                if not ok or frame is None:
                    continue
                if frame.shape[:2] != (HEIGHT, WIDTH):
                    frame = cv2.resize(frame, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self._put(rgb)
        except Exception as e:
            self.error = e
        finally:
            if self._cap is not None:
                try:
                    self._cap.release()
                except Exception:
                    pass

    def stop(self) -> None:
        self._stop_event.set()
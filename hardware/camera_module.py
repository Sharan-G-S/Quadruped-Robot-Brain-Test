"""
camera_module.py — OpenCV Camera Capture / Simulation
=====================================================
Captures frames for LLM vision processing and dashboard streaming.
"""

import logging
import time
import base64
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class CameraModule:
    """Camera capture with OpenCV, base64 encoding, and simulation fallback."""

    def __init__(self, config: dict):
        self.device_id = config.get("device_id", 0)
        self.width = config.get("width", 640)
        self.height = config.get("height", 480)
        self.fps = config.get("fps", 30)
        self.simulation = False
        self.cap = None
        self.cv2 = None
        self._last_frame = None
        self._count = 0

        self._init_camera()

    def _init_camera(self):
        try:
            import cv2
            self.cv2 = cv2
            self.cap = cv2.VideoCapture(self.device_id)
            if not self.cap.isOpened():
                raise RuntimeError("Camera could not be opened")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            logger.info(f"Camera: {self.width}x{self.height} @ {self.fps}fps")
        except Exception as e:
            logger.warning(f"Camera unavailable ({e}), using simulation")
            self.simulation = True
            try:
                import cv2
                self.cv2 = cv2
            except ImportError:
                self.cv2 = None

    # ── Capture ─────────────────────────────────────────────────

    def capture_frame(self) -> Optional[np.ndarray]:
        """Capture a BGR frame (np.ndarray) or None on failure."""
        self._count += 1
        if self.simulation:
            return self._sim_frame()
        try:
            ret, frame = self.cap.read()
            if ret:
                self._last_frame = frame
                return frame
            return self._last_frame
        except Exception as e:
            logger.error(f"Capture error: {e}")
            return self._last_frame

    def _sim_frame(self) -> np.ndarray:
        """Generate a synthetic scene for simulation."""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        frame[:] = (40, 35, 30)

        if self.cv2:
            cv2 = self.cv2
            t = self._count * 0.05

            # Floor grid
            for i in range(0, self.width, 40):
                xo = int(10 * np.sin(t + i * 0.01))
                cv2.line(frame, (i + xo, self.height // 2),
                         (i + xo * 2, self.height), (60, 55, 50), 1)

            # Moving ball (orange)
            cx = int(self.width / 2 + 150 * np.sin(t * 0.7))
            cy = int(self.height / 2 + 50 * np.cos(t * 0.5))
            cv2.circle(frame, (cx, cy), 30, (0, 120, 255), -1)
            cv2.circle(frame, (cx, cy), 32, (0, 180, 255), 2)

            # Obstacle box (green)
            ox = int(self.width * 0.75 + 50 * np.sin(t * 0.3))
            cv2.rectangle(frame, (ox, 200), (ox + 60, 350), (0, 80, 0), -1)
            cv2.rectangle(frame, (ox, 200), (ox + 60, 350), (0, 150, 0), 2)

            # HUD
            cv2.putText(frame, "[SIM] QuadBot-AI Camera", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 200), 1)
            cv2.putText(frame, f"Frame: {self._count}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)
            cv2.putText(frame, f"{self.width}x{self.height}",
                        (self.width - 110, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        self._last_frame = frame
        return frame

    # ── Encoding ────────────────────────────────────────────────

    def frame_to_base64(self, frame: Optional[np.ndarray] = None) -> Optional[str]:
        """Encode frame as base64 JPEG (for LLM API)."""
        f = frame if frame is not None else self._last_frame
        if f is None or self.cv2 is None:
            return None
        try:
            _, buf = self.cv2.imencode(".jpg", f, [self.cv2.IMWRITE_JPEG_QUALITY, 85])
            return base64.b64encode(buf).decode("utf-8")
        except Exception as e:
            logger.error(f"Encode error: {e}")
            return None

    def frame_to_jpeg_bytes(self, frame: Optional[np.ndarray] = None) -> Optional[bytes]:
        """Encode frame as JPEG bytes (for WebSocket streaming)."""
        f = frame if frame is not None else self._last_frame
        if f is None or self.cv2 is None:
            return None
        try:
            _, buf = self.cv2.imencode(".jpg", f, [self.cv2.IMWRITE_JPEG_QUALITY, 70])
            return buf.tobytes()
        except Exception as e:
            logger.error(f"JPEG error: {e}")
            return None

    # ── Utilities ───────────────────────────────────────────────

    def get_resolution(self) -> Tuple[int, int]:
        return (self.width, self.height)

    def release(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
            logger.info("Camera released")

    def get_telemetry(self) -> dict:
        return {
            "simulation": self.simulation,
            "resolution": f"{self.width}x{self.height}",
            "fps": self.fps, "frames": self._count,
        }

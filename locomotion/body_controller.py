"""
body_controller.py — High-Level Body Pose & Motion Controller
==============================================================
Manages body height, lean, and translates gait foot positions through IK to servo commands.
"""

import logging
import math
import time
from typing import Dict, List, Optional

from locomotion.kinematics import InverseKinematics
from locomotion.gait_controller import GaitController, GaitType

logger = logging.getLogger(__name__)


class BodyController:
    """
    Orchestrates the locomotion pipeline:
      GaitController (foot positions) → InverseKinematics → ServoDriver
    Also manages body tilt compensation and pose adjustments.
    """

    def __init__(self, gait_config: dict, body_config: dict, leg_config: dict):
        self.ik = InverseKinematics(leg_config)
        self.gait = GaitController(gait_config, body_config)

        self.default_height = body_config.get("default_height", 150.0)
        self.body_length = body_config.get("length", 200.0)
        self.body_width = body_config.get("width", 110.0)

        # Body pose adjustments
        self._height_offset = 0.0
        self._roll_lean = 0.0   # degrees
        self._pitch_lean = 0.0  # degrees
        self._yaw_lean = 0.0    # degrees

        logger.info("🦿 BodyController initialized")

    # ── Commands ────────────────────────────────────────────────

    def stand(self):
        self.gait.set_gait(GaitType.STAND)

    def sit(self):
        self.gait.set_gait(GaitType.SIT)

    def lay_down(self):
        self.gait.set_gait(GaitType.LAY_DOWN)

    def walk(self, speed: float = 0.3):
        self.gait.set_gait(GaitType.WALK, speed)

    def trot(self, speed: float = 0.5):
        self.gait.set_gait(GaitType.TROT, speed)

    def turn_left(self, speed: float = 0.4):
        self.gait.set_gait(GaitType.TURN_LEFT, speed)

    def turn_right(self, speed: float = 0.4):
        self.gait.set_gait(GaitType.TURN_RIGHT, speed)

    def side_left(self, speed: float = 0.3):
        self.gait.set_gait(GaitType.SIDE_LEFT, speed)

    def side_right(self, speed: float = 0.3):
        self.gait.set_gait(GaitType.SIDE_RIGHT, speed)

    def set_height(self, offset_mm: float):
        """Adjust body height (positive = taller)."""
        self._height_offset = max(-50, min(50, offset_mm))

    def set_lean(self, roll: float = 0, pitch: float = 0, yaw: float = 0):
        """Set body lean in degrees (for expressive poses)."""
        self._roll_lean = max(-15, min(15, roll))
        self._pitch_lean = max(-15, min(15, pitch))
        self._yaw_lean = max(-15, min(15, yaw))

    # ── Main Update Loop ────────────────────────────────────────

    def update(self) -> Dict[str, List[float]]:
        """
        Compute servo angles for all legs at the current moment.

        Returns:
            {"FR": [hip, upper, lower], "FL": [...], ...}
        """
        # Step 1: Get foot target positions from gait controller
        foot_positions = self.gait.get_foot_positions()

        # Step 2: Apply body pose adjustments
        adjusted = self._apply_body_pose(foot_positions)

        # Step 3: Run IK to get servo angles
        servo_angles = self.ik.solve_all_legs(adjusted)

        return servo_angles

    def _apply_body_pose(self, positions: dict) -> dict:
        """
        Apply height offset and body lean to foot positions.
        This shifts the body relative to the feet (feet stay on ground).
        """
        adjusted = {}
        roll_rad = math.radians(self._roll_lean)
        pitch_rad = math.radians(self._pitch_lean)

        for leg, (x, y, z) in positions.items():
            # Height offset
            z_new = z - self._height_offset

            # Roll lean: tilt left/right
            # Left legs go down, right legs go up (or vice versa)
            if leg.endswith("R"):
                z_new += self.body_width / 2 * math.sin(roll_rad)
            else:
                z_new -= self.body_width / 2 * math.sin(roll_rad)

            # Pitch lean: tilt forward/backward
            if leg.startswith("F"):
                z_new += self.body_length / 2 * math.sin(pitch_rad)
            else:
                z_new -= self.body_length / 2 * math.sin(pitch_rad)

            adjusted[leg] = (x, y, max(30, z_new))  # Min height safety

        return adjusted

    # ── IMU Stabilization ───────────────────────────────────────

    def compensate_tilt(self, roll: float, pitch: float,
                        gain: float = 0.3):
        """
        Auto-stabilize body using IMU feedback.
        Applies counter-lean to keep the body level.
        """
        self._roll_lean = -roll * gain
        self._pitch_lean = -pitch * gain

    # ── Action Execution ────────────────────────────────────────

    def execute_action(self, action: dict):
        """
        Execute a high-level action command from the brain.

        Supported actions:
          {"action": "walk_forward", "speed": 0.5}
          {"action": "turn_left", "speed": 0.4}
          {"action": "stop"}
          {"action": "sit"}
          {"action": "stand"}
          {"action": "set_height", "offset": 20}
          {"action": "lean", "roll": 5, "pitch": 0}
        """
        act = action.get("action", "stop")
        speed = action.get("speed", 0.5)

        action_map = {
            "walk_forward": lambda: self.walk(speed),
            "walk": lambda: self.walk(speed),
            "trot": lambda: self.trot(speed),
            "run": lambda: self.trot(min(1.0, speed * 1.5)),
            "turn_left": lambda: self.turn_left(speed),
            "turn_right": lambda: self.turn_right(speed),
            "side_left": lambda: self.side_left(speed),
            "side_right": lambda: self.side_right(speed),
            "stop": self.stand,
            "stand": self.stand,
            "sit": self.sit,
            "lay_down": self.lay_down,
            "set_height": lambda: self.set_height(action.get("offset", 0)),
            "lean": lambda: self.set_lean(
                action.get("roll", 0),
                action.get("pitch", 0),
                action.get("yaw", 0),
            ),
        }

        handler = action_map.get(act)
        if handler:
            handler()
            logger.info(f"🎯 Action executed: {act}")
        else:
            logger.warning(f"Unknown action: {act}")

    # ── Telemetry ───────────────────────────────────────────────

    def get_telemetry(self) -> dict:
        return {
            "gait": self.gait.get_telemetry(),
            "height_offset": self._height_offset,
            "roll_lean": round(self._roll_lean, 2),
            "pitch_lean": round(self._pitch_lean, 2),
            "yaw_lean": round(self._yaw_lean, 2),
        }

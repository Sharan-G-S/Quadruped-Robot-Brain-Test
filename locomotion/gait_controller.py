"""
gait_controller.py — Quadruped Gait Pattern Generator
=======================================================
Generates time-phased foot trajectories for walk, trot, turn, etc.
Each gait defines the phase offset and foot path for all 4 legs.
"""

import math
import time
import logging
from typing import Dict, List, Tuple
from enum import Enum

logger = logging.getLogger(__name__)


class GaitType(Enum):
    STAND = "stand"
    SIT = "sit"
    LAY_DOWN = "lay_down"
    WALK = "walk"
    TROT = "trot"
    TURN_LEFT = "turn_left"
    TURN_RIGHT = "turn_right"
    SIDE_LEFT = "side_left"
    SIDE_RIGHT = "side_right"


# Phase offsets per leg for each gait pattern
# 0.0–1.0 represents position in the gait cycle
GAIT_PHASES = {
    GaitType.WALK: {"FR": 0.0, "RL": 0.25, "FL": 0.5, "RR": 0.75},
    GaitType.TROT: {"FR": 0.0, "RL": 0.0, "FL": 0.5, "RR": 0.5},
}

LEG_NAMES = ["FR", "FL", "RR", "RL"]


class GaitController:
    """Generates foot trajectories for various gaits."""

    def __init__(self, config: dict, body_config: dict):
        self.step_height = config.get("step_height", 30.0)
        self.step_length = config.get("step_length", 60.0)
        self.cycle_time = config.get("cycle_time", 0.8)
        self.body_swing = config.get("body_swing", 5.0)

        self.body_length = body_config.get("length", 200.0)
        self.body_width = body_config.get("width", 110.0)
        self.default_height = body_config.get("default_height", 150.0)

        self.current_gait = GaitType.STAND
        self._start_time = time.time()
        self.speed = config.get("trot_speed", 0.5)

        # Default standing positions (x, y, z) per leg relative to hip
        self.stand_positions = self._compute_stand_positions()

    def _compute_stand_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Neutral standing foot positions."""
        half_l = self.body_length / 2
        half_w = self.body_width / 2
        h = self.default_height
        return {
            "FR": (half_l,  half_w, h),
            "FL": (half_l, -half_w, h),
            "RR": (-half_l,  half_w, h),
            "RL": (-half_l, -half_w, h),
        }

    # ── Set Gait ────────────────────────────────────────────────

    def set_gait(self, gait: GaitType, speed: float = 0.5):
        if gait != self.current_gait:
            logger.info(f"Gait -> {gait.value} (speed={speed:.1f})")
            self.current_gait = gait
            self.speed = max(0.1, min(1.0, speed))
            self._start_time = time.time()

    # ── Generate Foot Positions ─────────────────────────────────

    def get_foot_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """
        Get current foot target positions for all 4 legs
        based on the active gait and elapsed time.
        """
        if self.current_gait == GaitType.STAND:
            return dict(self.stand_positions)
        elif self.current_gait == GaitType.SIT:
            return self._sit_positions()
        elif self.current_gait == GaitType.LAY_DOWN:
            return self._lay_positions()
        elif self.current_gait in (GaitType.WALK, GaitType.TROT):
            return self._locomotion_positions()
        elif self.current_gait in (GaitType.TURN_LEFT, GaitType.TURN_RIGHT):
            return self._turn_positions()
        elif self.current_gait in (GaitType.SIDE_LEFT, GaitType.SIDE_RIGHT):
            return self._side_step_positions()
        else:
            return dict(self.stand_positions)

    def _locomotion_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Walk/trot gait — elliptical foot trajectory."""
        elapsed = time.time() - self._start_time
        phase_dict = GAIT_PHASES.get(self.current_gait, GAIT_PHASES[GaitType.TROT])
        cycle = self.cycle_time / self.speed
        positions = {}

        for leg in LEG_NAMES:
            base = self.stand_positions[leg]
            phase_offset = phase_dict.get(leg, 0.0)
            phase = ((elapsed / cycle) + phase_offset) % 1.0

            x_off, z_off = self._swing_stance_trajectory(phase)
            positions[leg] = (
                base[0] + x_off,
                base[1],
                base[2] + z_off,
            )

        return positions

    def _swing_stance_trajectory(self, phase: float) -> Tuple[float, float]:
        """
        Compute (x_offset, z_offset) for a single leg at given phase.
        phase 0.0--0.5: swing (leg in air, moving forward)
        phase 0.5--1.0: stance (leg on ground, pushing backward)
        """
        half_step = self.step_length / 2

        if phase < 0.5:
            # Swing phase: leg lifts and moves forward
            t = phase / 0.5  # 0->1
            x_off = -half_step + self.step_length * t
            z_off = -self.step_height * math.sin(t * math.pi)
        else:
            # Stance phase: leg on ground, slides backward
            t = (phase - 0.5) / 0.5  # 0->1
            x_off = half_step - self.step_length * t
            z_off = 0.0

        return (x_off, z_off)

    def _turn_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Turning gait — legs on one side step forward, other backward."""
        elapsed = time.time() - self._start_time
        cycle = self.cycle_time / self.speed
        positions = {}
        direction = 1.0 if self.current_gait == GaitType.TURN_RIGHT else -1.0

        for leg in LEG_NAMES:
            base = self.stand_positions[leg]
            phase_offset = GAIT_PHASES[GaitType.TROT].get(leg, 0.0)
            phase = ((elapsed / cycle) + phase_offset) % 1.0

            x_off, z_off = self._swing_stance_trajectory(phase)

            # Flip direction for one side
            side_mult = direction if leg.endswith("R") else -direction
            positions[leg] = (
                base[0] + x_off * side_mult * 0.5,
                base[1],
                base[2] + z_off,
            )

        return positions

    def _side_step_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Side-stepping gait."""
        elapsed = time.time() - self._start_time
        cycle = self.cycle_time / self.speed
        direction = 1.0 if self.current_gait == GaitType.SIDE_RIGHT else -1.0
        positions = {}

        for leg in LEG_NAMES:
            base = self.stand_positions[leg]
            phase_offset = GAIT_PHASES[GaitType.TROT].get(leg, 0.0)
            phase = ((elapsed / cycle) + phase_offset) % 1.0

            y_off, z_off = self._swing_stance_trajectory(phase)
            positions[leg] = (
                base[0],
                base[1] + y_off * direction * 0.5,
                base[2] + z_off,
            )

        return positions

    def _sit_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Sitting pose: rear legs tucked, front legs straight."""
        h = self.default_height
        return {
            "FR": self.stand_positions["FR"],
            "FL": self.stand_positions["FL"],
            "RR": (self.stand_positions["RR"][0] + 20,
                   self.stand_positions["RR"][1], h * 0.5),
            "RL": (self.stand_positions["RL"][0] + 20,
                   self.stand_positions["RL"][1], h * 0.5),
        }

    def _lay_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Laying down pose: all legs tucked under body."""
        return {
            leg: (base[0], base[1], self.default_height * 0.3)
            for leg, base in self.stand_positions.items()
        }

    # ── Telemetry ───────────────────────────────────────────────

    def get_telemetry(self) -> dict:
        return {
            "gait": self.current_gait.value,
            "speed": round(self.speed, 2),
            "step_height": self.step_height,
            "step_length": self.step_length,
            "cycle_time": self.cycle_time,
        }

"""
kinematics.py — 3-DOF Inverse Kinematics for Quadruped Legs
=============================================================
Computes servo angles (hip, upper, lower) from desired foot position (x, y, z).

Leg geometry:
  - Hip joint:   rotates the leg laterally (abduction/adduction)
  - Upper joint: rotates the femur (thigh)
  - Lower joint: rotates the tibia (shin)

Coordinate frame (per leg, origin at hip mount):
  x = forward (+) / backward (-)
  y = outward (+) / inward (-)
  z = down (+) / up (-)
"""

import math
import logging
from typing import Tuple, Optional

import numpy as np

logger = logging.getLogger(__name__)


class InverseKinematics:
    """3-DOF inverse kinematics for one quadruped leg."""

    def __init__(self, config: dict):
        self.L_hip = config.get("hip_length", 50.0)      # mm
        self.L_upper = config.get("upper_length", 107.0)  # mm
        self.L_lower = config.get("lower_length", 130.0)  # mm

        # Pre-compute for range checks
        self.reach_max = self.L_upper + self.L_lower
        self.reach_min = abs(self.L_upper - self.L_lower)

    # ── Inverse Kinematics ──────────────────────────────────────

    def solve(self, x: float, y: float, z: float,
              mirror: bool = False) -> Optional[Tuple[float, float, float]]:
        """
        Compute joint angles for a foot at (x, y, z) relative to hip.

        Args:
            x: forward (+) / backward (-) in mm
            y: outward (+) / inward (-) in mm
            z: down (+) from hip in mm
            mirror: True for left-side legs (flips hip angle)

        Returns:
            (hip_angle, upper_angle, lower_angle) in degrees,
            or None if position is unreachable.
        """
        try:
            # ── Hip angle (lateral swing) ───────────────────────
            # Project leg onto YZ plane
            y_eff = y - (self.L_hip if not mirror else -self.L_hip)
            dyz = math.sqrt(y_eff**2 + z**2)

            hip_rad = math.atan2(y_eff, z)
            hip_deg = math.degrees(hip_rad)

            # ── Project onto the leg plane (XZ after hip rotation) ──
            L_proj = math.sqrt(max(0, dyz**2 - 0))  # distance in leg plane
            D = math.sqrt(x**2 + L_proj**2)

            # Range check
            if D > self.reach_max or D < self.reach_min:
                logger.debug(f"IK: position ({x:.1f}, {y:.1f}, {z:.1f}) "
                             f"unreachable (D={D:.1f})")
                # Clamp to nearest reachable
                D = max(self.reach_min + 1, min(self.reach_max - 1, D))

            # ── Lower joint angle (knee) via law of cosines ─────
            cos_lower = (self.L_upper**2 + self.L_lower**2 - D**2) / \
                        (2 * self.L_upper * self.L_lower)
            cos_lower = max(-1, min(1, cos_lower))
            lower_rad = math.acos(cos_lower)
            lower_deg = math.degrees(lower_rad)

            # ── Upper joint angle (shoulder) ────────────────────
            alpha = math.atan2(x, L_proj)
            cos_beta = (self.L_upper**2 + D**2 - self.L_lower**2) / \
                       (2 * self.L_upper * D)
            cos_beta = max(-1, min(1, cos_beta))
            beta = math.acos(cos_beta)

            upper_rad = alpha + beta
            upper_deg = math.degrees(upper_rad)

            # ── Convert to servo angles (0-180° range) ─────────
            hip_servo = 90.0 + hip_deg
            upper_servo = upper_deg
            lower_servo = lower_deg

            if mirror:
                hip_servo = 180.0 - hip_servo

            # Clamp to servo range
            hip_servo = max(0, min(180, hip_servo))
            upper_servo = max(0, min(180, upper_servo))
            lower_servo = max(0, min(180, lower_servo))

            return (hip_servo, upper_servo, lower_servo)

        except (ValueError, ZeroDivisionError) as e:
            logger.error(f"IK solve error at ({x}, {y}, {z}): {e}")
            return None

    # ── Forward Kinematics (for verification) ───────────────────

    def forward(self, hip_deg: float, upper_deg: float, lower_deg: float,
                mirror: bool = False) -> Tuple[float, float, float]:
        """
        Compute foot position (x, y, z) from joint angles.
        Used for verification and visualization.
        """
        hip_rad = math.radians(hip_deg - 90.0)
        if mirror:
            hip_rad = -hip_rad

        upper_rad = math.radians(upper_deg)
        lower_rad = math.radians(lower_deg)

        # Foot position in leg plane
        leg_z = self.L_upper * math.cos(upper_rad) + \
                self.L_lower * math.cos(upper_rad - (math.pi - lower_rad))
        leg_x = self.L_upper * math.sin(upper_rad) + \
                self.L_lower * math.sin(upper_rad - (math.pi - lower_rad))

        # Apply hip rotation
        z = leg_z * math.cos(hip_rad)
        y = leg_z * math.sin(hip_rad) + (self.L_hip if not mirror else -self.L_hip)
        x = leg_x

        return (x, y, z)

    # ── Batch Solve (all 4 legs) ────────────────────────────────

    def solve_all_legs(self, positions: dict) -> dict:
        """
        Solve IK for all 4 legs.

        Args:
            positions: {"FR": (x, y, z), "FL": (x, y, z),
                       "RR": (x, y, z), "RL": (x, y, z)}

        Returns:
            {"FR": (hip, upper, lower), ...} or None for unreachable legs.
        """
        result = {}
        for leg, (x, y, z) in positions.items():
            mirror = leg in ("FL", "RL")  # Left legs are mirrored
            angles = self.solve(x, y, z, mirror=mirror)
            if angles:
                result[leg] = list(angles)
            else:
                logger.warning(f"IK failed for {leg} at ({x:.1f}, {y:.1f}, {z:.1f})")
                result[leg] = [90.0, 90.0, 90.0]  # Safe default
        return result

    def workspace_check(self, x: float, y: float, z: float) -> bool:
        """Check if a position is within the leg's workspace."""
        y_eff = y - self.L_hip
        D = math.sqrt(x**2 + y_eff**2 + z**2)
        return self.reach_min < D < self.reach_max

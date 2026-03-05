"""
servo_driver.py — PCA9685 Servo Controller / Simulation
========================================================
Controls 12 servos (3-DOF × 4 legs) via PCA9685 I2C or simulation fallback.
"""

import logging
import math
import time
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class ServoDriver:
    """Drives servos through PCA9685 I2C PWM controller or simulation."""

    def __init__(self, config: dict):
        self.config = config
        self.frequency = config.get("frequency", 50)
        self.min_pulse = config.get("min_pulse", 500)
        self.max_pulse = config.get("max_pulse", 2500)
        self.min_angle = config.get("min_angle", 0)
        self.max_angle = config.get("max_angle", 180)
        self.offsets = config.get("offsets", [0] * 16)
        self.channels = config.get("channels", {})
        self.simulation = False
        self._current_angles: Dict[int, float] = {}

        self._init_driver()

    # ── Initialization ──────────────────────────────────────────

    def _init_driver(self):
        try:
            from adafruit_servokit import ServoKit
            self.kit = ServoKit(
                channels=16,
                address=self.config.get("i2c_address", 0x40),
                frequency=self.frequency,
            )
            for i in range(16):
                self.kit.servo[i].set_pulse_width_range(
                    self.min_pulse, self.max_pulse
                )
            logger.info("PCA9685 servo driver initialized (I2C)")
        except Exception as e:
            logger.warning(f"PCA9685 unavailable ({e}), using simulation")
            self.simulation = True
            self.kit = None

    # ── Core Servo Control ──────────────────────────────────────

    def set_angle(self, channel: int, angle: float):
        """Set a single servo angle (degrees) with calibration offset."""
        if not 0 <= channel <= 15:
            logger.error(f"Invalid servo channel: {channel}")
            return

        offset = self.offsets[channel] if channel < len(self.offsets) else 0
        adjusted = max(self.min_angle, min(self.max_angle, angle + offset))
        self._current_angles[channel] = adjusted

        if self.simulation:
            logger.debug(f"[SIM] CH{channel:02d} → {adjusted:.1f}°")
        else:
            try:
                self.kit.servo[channel].angle = adjusted
            except Exception as e:
                logger.error(f"Servo write CH{channel}: {e}")

    def set_leg_angles(self, leg: str, hip: float, upper: float, lower: float):
        """Set all 3 servo angles for a named leg (FR, FL, RR, RL)."""
        if leg not in self.channels:
            logger.error(f"Unknown leg: {leg}")
            return
        chs = self.channels[leg]
        self.set_angle(chs[0], hip)
        self.set_angle(chs[1], upper)
        self.set_angle(chs[2], lower)

    def set_all_legs(self, angles: Dict[str, List[float]]):
        """Set angles for all legs: {"FR": [hip, upper, lower], ...}"""
        for leg, a in angles.items():
            if len(a) == 3:
                self.set_leg_angles(leg, *a)

    # ── Utilities ───────────────────────────────────────────────

    def center_all(self):
        """Move all servos to 90° center."""
        center = (self.min_angle + self.max_angle) / 2
        for chs in self.channels.values():
            for ch in chs:
                self.set_angle(ch, center)
        logger.info(f"All servos centered to {center}°")

    def relax(self):
        """Disable PWM to all servos (go limp)."""
        if not self.simulation and self.kit:
            for i in range(16):
                try:
                    self.kit.servo[i].angle = None
                except Exception:
                    pass
        logger.info("Servos relaxed")

    def smooth_move(self, channel: int, target: float,
                    duration: float = 0.5, steps: int = 20):
        """Sinusoidal ease-in-out interpolation to target angle."""
        current = self._current_angles.get(channel, 90.0)
        dt = duration / steps
        for i in range(1, steps + 1):
            t = (1 - math.cos((i / steps) * math.pi)) / 2
            self.set_angle(channel, current + (target - current) * t)
            time.sleep(dt)

    def get_current_angles(self) -> Dict[int, float]:
        return dict(self._current_angles)

    def get_telemetry(self) -> dict:
        return {
            "simulation": self.simulation,
            "channels": dict(self._current_angles),
            "legs": {
                leg: [self._current_angles.get(ch, 90.0) for ch in chs]
                for leg, chs in self.channels.items()
            },
        }

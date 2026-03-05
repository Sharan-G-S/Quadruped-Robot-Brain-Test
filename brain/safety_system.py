"""
safety_system.py — Robot Safety & Emergency Protection
========================================================
Monitors IMU, distance, and battery sensors to detect falls,
obstacles, and unsafe conditions. Overrides all other commands.
"""

import logging
import time
from typing import Optional, Dict, Callable

logger = logging.getLogger(__name__)


class SafetySystem:
    """
    Monitors sensor data for unsafe conditions and triggers emergency stops.
    Safety always has highest priority — overrides brain and user commands.
    """

    def __init__(self, config: dict):
        self.max_tilt = config.get("max_tilt_degrees", 45.0)
        self.max_accel_g = config.get("max_accel_g", 3.0)
        self.min_battery = config.get("min_battery_voltage", 6.0)
        self.enabled = config.get("emergency_stop_enabled", True)

        self._is_safe = True
        self._alerts: list = []
        self._last_check = time.time()
        self._fall_detected = False
        self._obstacle_emergency = False
        self._battery_low = False
        self._emergency_callback: Optional[Callable] = None
        self._consecutive_tilt_count = 0

        logger.info(f"Safety system initialized (enabled={self.enabled})")

    def set_emergency_callback(self, callback: Callable):
        """Set callback for when emergency is triggered."""
        self._emergency_callback = callback

    # ── Main Check ──────────────────────────────────────────────

    def check(self, imu_data: Optional[Dict] = None,
              distance_data: Optional[Dict] = None,
              battery_voltage: Optional[float] = None) -> bool:
        """
        Run all safety checks. Returns True if safe, False if emergency.

        This should be called every control loop iteration.
        """
        if not self.enabled:
            return True

        self._alerts = []
        self._is_safe = True

        # Check IMU (fall & tilt detection)
        if imu_data:
            self._check_tilt(imu_data)
            self._check_fall(imu_data)

        # Check distance (obstacle emergency)
        if distance_data:
            self._check_obstacle(distance_data)

        # Check battery
        if battery_voltage is not None:
            self._check_battery(battery_voltage)

        # Trigger emergency if unsafe
        if not self._is_safe and self._emergency_callback:
            reason = "; ".join(self._alerts)
            logger.critical(f"SAFETY ALERT: {reason}")
            self._emergency_callback(reason)

        self._last_check = time.time()
        return self._is_safe

    # ── Individual Checks ───────────────────────────────────────

    def _check_tilt(self, imu: Dict):
        """Check if robot is tilted beyond safe limits."""
        roll = abs(imu.get("roll", 0))
        pitch = abs(imu.get("pitch", 0))

        if roll > self.max_tilt or pitch > self.max_tilt:
            self._consecutive_tilt_count += 1
            # Require sustained tilt to avoid false positives from vibration
            if self._consecutive_tilt_count >= 3:
                self._is_safe = False
                self._alerts.append(
                    f"Excessive tilt: roll={roll:.1f}° pitch={pitch:.1f}°"
                )
        else:
            self._consecutive_tilt_count = 0

    def _check_fall(self, imu: Dict):
        """Check for sudden acceleration indicating a fall."""
        accel_mag = (imu.get("accel_x", 0)**2 +
                     imu.get("accel_y", 0)**2 +
                     imu.get("accel_z", 0)**2) ** 0.5

        # Free-fall: very low acceleration
        if accel_mag < 2.0:
            self._fall_detected = True
            self._is_safe = False
            self._alerts.append(f"Free-fall detected (accel={accel_mag:.1f} m/s²)")

        # Impact: very high acceleration
        elif accel_mag > self.max_accel_g * 9.81:
            self._fall_detected = True
            self._is_safe = False
            self._alerts.append(
                f"Impact detected (accel={accel_mag:.1f} m/s², "
                f"threshold={self.max_accel_g * 9.81:.1f})"
            )
        else:
            self._fall_detected = False

    def _check_obstacle(self, dist_data: Dict):
        """Check for dangerously close obstacles."""
        distance = dist_data.get("distance_cm", 999)
        # Critical: < 10cm = immediate stop
        if distance < 10:
            self._obstacle_emergency = True
            self._is_safe = False
            self._alerts.append(f"Obstacle critically close: {distance:.0f}cm")
        else:
            self._obstacle_emergency = False

    def _check_battery(self, voltage: float):
        """Check battery voltage."""
        if voltage < self.min_battery:
            self._battery_low = True
            self._is_safe = False
            self._alerts.append(
                f"Battery low: {voltage:.1f}V (min={self.min_battery}V)"
            )
        else:
            self._battery_low = False

    # ── Status ──────────────────────────────────────────────────

    @property
    def is_safe(self) -> bool:
        return self._is_safe

    @property
    def alerts(self) -> list:
        return list(self._alerts)

    def get_telemetry(self) -> dict:
        return {
            "enabled": self.enabled,
            "is_safe": self._is_safe,
            "fall_detected": self._fall_detected,
            "obstacle_emergency": self._obstacle_emergency,
            "battery_low": self._battery_low,
            "active_alerts": self._alerts,
            "max_tilt_deg": self.max_tilt,
        }

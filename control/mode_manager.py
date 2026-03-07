"""
mode_manager.py -- Control Mode Manager
==========================================
Manages switching between MANUAL, AUTONOMOUS, and HYBRID control modes.
In MANUAL mode, the robot responds only to direct user commands.
In AUTONOMOUS mode, the LLM/vision pipeline drives the robot.
In HYBRID mode, both are active but manual commands take priority.
"""

import logging
import time
from enum import Enum
from typing import Optional

logger = logging.getLogger(__name__)


class ControlMode(Enum):
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    HYBRID = "hybrid"


class ModeManager:
    """Central controller for switching between manual and autonomous modes."""

    def __init__(self, config: dict):
        default = config.get("default_mode", "autonomous")
        self.allow_switch = config.get("allow_mode_switch", True)
        self.manual_timeout = config.get("manual_timeout_sec", 0)

        self._mode = ControlMode(default)
        self._prev_mode: Optional[ControlMode] = None
        self._mode_start_time = time.time()
        self._switch_count = 0

        logger.info(f"ModeManager initialized -> {self._mode.value}")

    # -- Mode Queries -------------------------------------------------------

    @property
    def mode(self) -> ControlMode:
        return self._mode

    @property
    def is_manual(self) -> bool:
        return self._mode == ControlMode.MANUAL

    @property
    def is_autonomous(self) -> bool:
        return self._mode == ControlMode.AUTONOMOUS

    @property
    def is_hybrid(self) -> bool:
        return self._mode == ControlMode.HYBRID

    @property
    def should_run_autonomy(self) -> bool:
        """Whether the autonomous pipeline (LLM/vision) should execute."""
        return self._mode in (ControlMode.AUTONOMOUS, ControlMode.HYBRID)

    @property
    def should_accept_manual(self) -> bool:
        """Whether manual commands should be accepted."""
        return self._mode in (ControlMode.MANUAL, ControlMode.HYBRID)

    @property
    def mode_duration(self) -> float:
        return time.time() - self._mode_start_time

    # -- Mode Switching -----------------------------------------------------

    def switch_mode(self, new_mode: str) -> bool:
        """
        Switch to a new control mode.

        Args:
            new_mode: 'manual', 'autonomous', or 'hybrid'

        Returns:
            True if switch was successful.
        """
        if not self.allow_switch:
            logger.warning("Mode switching is disabled in config")
            return False

        try:
            target = ControlMode(new_mode.lower())
        except ValueError:
            logger.warning(f"Invalid control mode: {new_mode}")
            return False

        if target == self._mode:
            return True

        self._prev_mode = self._mode
        self._mode = target
        self._mode_start_time = time.time()
        self._switch_count += 1

        logger.info(
            f"Control mode: {self._prev_mode.value} -> {target.value}"
        )
        return True

    # -- Timeout Check (optional auto-return to autonomous) -----------------

    def check_timeout(self) -> bool:
        """
        If manual_timeout_sec > 0 and we are in MANUAL mode,
        auto-switch back to AUTONOMOUS after timeout.
        Returns True if a timeout switch occurred.
        """
        if (
            self.manual_timeout > 0
            and self._mode == ControlMode.MANUAL
            and self.mode_duration > self.manual_timeout
        ):
            logger.info("Manual mode timed out, switching to autonomous")
            self.switch_mode("autonomous")
            return True
        return False

    # -- Telemetry ----------------------------------------------------------

    def get_telemetry(self) -> dict:
        return {
            "mode": self._mode.value,
            "previous_mode": self._prev_mode.value if self._prev_mode else None,
            "duration": round(self.mode_duration, 1),
            "switch_count": self._switch_count,
            "allow_switch": self.allow_switch,
        }

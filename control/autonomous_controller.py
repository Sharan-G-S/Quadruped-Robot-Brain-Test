"""
autonomous_controller.py -- Autonomous Control Handler
========================================================
Wraps the existing LLM/vision-based autonomous pipeline into a
controller that can be enabled/disabled based on the control mode.
"""

import logging
import time
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class AutonomousController:
    """
    Manages the autonomous (LLM/vision) control pipeline.
    Can be enabled or disabled by the ModeManager.
    """

    def __init__(self):
        self._enabled = True
        self._last_decision: Optional[Dict] = None
        self._decision_count = 0
        self._last_decision_time = 0.0

        logger.info("AutonomousController initialized")

    # -- Enable / Disable --------------------------------------------------

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self):
        """Enable autonomous decision-making."""
        if not self._enabled:
            self._enabled = True
            logger.info("Autonomous control ENABLED")

    def disable(self):
        """Disable autonomous decision-making."""
        if self._enabled:
            self._enabled = False
            logger.info("Autonomous control DISABLED")

    # -- Decision Processing ------------------------------------------------

    def process_decision(self, vision_result: Optional[Dict],
                         sensor_data: Optional[Dict],
                         current_state: str,
                         decision_engine) -> Optional[Dict[str, Any]]:
        """
        Run the autonomous decision pipeline if enabled.

        Args:
            vision_result: Latest vision analysis result
            sensor_data: Latest sensor data
            current_state: Current robot state string
            decision_engine: The DecisionEngine instance

        Returns:
            Action dict or None if disabled.
        """
        if not self._enabled:
            return None

        decision = decision_engine.decide(
            vision_result=vision_result,
            sensor_data=sensor_data,
            current_state=current_state,
        )

        self._last_decision = decision
        self._decision_count += 1
        self._last_decision_time = time.time()

        return decision

    # -- State Queries -----------------------------------------------------

    @property
    def last_decision(self) -> Optional[Dict]:
        return self._last_decision

    @property
    def time_since_decision(self) -> float:
        if self._last_decision_time == 0:
            return float("inf")
        return time.time() - self._last_decision_time

    def get_telemetry(self) -> dict:
        return {
            "enabled": self._enabled,
            "decision_count": self._decision_count,
            "last_action": (
                self._last_decision.get("action")
                if self._last_decision else None
            ),
            "time_since_decision": round(self.time_since_decision, 1),
        }

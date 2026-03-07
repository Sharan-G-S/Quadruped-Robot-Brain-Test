"""
manual_controller.py -- Manual Control Handler
=================================================
Processes direct user commands (dashboard buttons, keyboard, joystick)
into structured robot actions without involving the LLM pipeline.
"""

import logging
import time
from typing import Optional, Dict, Any, List
from collections import deque

logger = logging.getLogger(__name__)


class ManualController:
    """
    Handles manual control input: queues user commands and converts
    them to robot actions without LLM processing.
    """

    def __init__(self):
        self._command_queue: deque = deque(maxlen=20)
        self._last_action: Optional[Dict] = None
        self._action_count = 0
        self._last_input_time = 0.0

        logger.info("ManualController initialized")

    # -- Command Submission -------------------------------------------------

    def submit_command(self, action: str, speed: float = 0.5,
                       duration: float = 0) -> Dict[str, Any]:
        """
        Submit a direct manual command.

        Args:
            action: Action name (walk_forward, turn_left, stop, etc.)
            speed: Movement speed 0.1-1.0
            duration: Duration in seconds (0 = continuous)

        Returns:
            Structured action dict ready for body_controller.
        """
        speed = max(0.1, min(1.0, speed))

        command = {
            "action": action,
            "speed": speed,
            "duration": duration,
            "reasoning": f"Manual: {action}",
            "priority": "high",
            "source": "manual",
            "timestamp": time.time(),
        }

        self._command_queue.append(command)
        self._last_action = command
        self._action_count += 1
        self._last_input_time = time.time()

        logger.info(f"Manual command: {action} (speed={speed:.1f})")
        return command

    def submit_joystick(self, x: float, y: float,
                        speed: float = 0.5) -> Dict[str, Any]:
        """
        Convert joystick X/Y input (-1.0 to 1.0) to a robot action.

        Args:
            x: Left/right axis (-1.0 = left, 1.0 = right)
            y: Forward/backward axis (-1.0 = backward, 1.0 = forward)
            speed: Base speed multiplier
        """
        deadzone = 0.15

        if abs(x) < deadzone and abs(y) < deadzone:
            return self.submit_command("stop", 0)

        # Determine primary action from joystick direction
        if abs(y) > abs(x):
            if y > 0:
                action = "walk_forward"
                cmd_speed = speed * abs(y)
            else:
                action = "walk_forward"
                cmd_speed = speed * abs(y) * -1  # Negative for backward
        else:
            if x > 0:
                action = "turn_right"
            else:
                action = "turn_left"
            cmd_speed = speed * abs(x)

        return self.submit_command(action, abs(cmd_speed))

    # -- Queue Processing ---------------------------------------------------

    def get_pending_action(self) -> Optional[Dict[str, Any]]:
        """
        Get the next pending manual action from the queue.
        Returns None if queue is empty.
        """
        if self._command_queue:
            return self._command_queue.popleft()
        return None

    def has_pending(self) -> bool:
        """Check if there are pending manual commands."""
        return len(self._command_queue) > 0

    def clear_queue(self):
        """Clear all pending commands."""
        self._command_queue.clear()
        logger.info("Manual command queue cleared")

    # -- State Queries ------------------------------------------------------

    @property
    def last_action(self) -> Optional[Dict]:
        return self._last_action

    @property
    def time_since_input(self) -> float:
        """Seconds since last manual input."""
        if self._last_input_time == 0:
            return float("inf")
        return time.time() - self._last_input_time

    def get_telemetry(self) -> dict:
        return {
            "queue_size": len(self._command_queue),
            "last_action": self._last_action.get("action") if self._last_action else None,
            "action_count": self._action_count,
            "time_since_input": round(self.time_since_input, 1),
        }

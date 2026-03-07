"""
state_machine.py — Robot Behavior State Machine
=================================================
Manages robot states and transitions driven by sensor data,
LLM decisions, and user commands.
"""

import logging
import time
from enum import Enum
from typing import Optional, Dict, Callable, List

logger = logging.getLogger(__name__)


class RobotState(Enum):
    IDLE = "idle"
    STANDING = "standing"
    WALKING = "walking"
    TROTTING = "trotting"
    TURNING = "turning"
    EXPLORING = "exploring"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    FOLLOWING = "following"
    SITTING = "sitting"
    LAYING_DOWN = "laying_down"
    MANUAL_CONTROL = "manual_control"
    EMERGENCY_STOP = "emergency_stop"


# Valid state transitions: {from_state: [allowed_to_states]}
TRANSITIONS = {
    RobotState.IDLE: [
        RobotState.STANDING, RobotState.EMERGENCY_STOP,
    ],
    RobotState.STANDING: [
        RobotState.WALKING, RobotState.TROTTING, RobotState.TURNING,
        RobotState.EXPLORING, RobotState.SITTING, RobotState.LAYING_DOWN,
        RobotState.MANUAL_CONTROL, RobotState.IDLE, RobotState.EMERGENCY_STOP,
    ],
    RobotState.WALKING: [
        RobotState.STANDING, RobotState.TROTTING, RobotState.TURNING,
        RobotState.OBSTACLE_AVOIDANCE, RobotState.EMERGENCY_STOP,
    ],
    RobotState.TROTTING: [
        RobotState.STANDING, RobotState.WALKING, RobotState.TURNING,
        RobotState.OBSTACLE_AVOIDANCE, RobotState.EMERGENCY_STOP,
    ],
    RobotState.TURNING: [
        RobotState.STANDING, RobotState.WALKING, RobotState.TROTTING,
        RobotState.OBSTACLE_AVOIDANCE, RobotState.EMERGENCY_STOP,
    ],
    RobotState.EXPLORING: [
        RobotState.STANDING, RobotState.WALKING, RobotState.TURNING,
        RobotState.OBSTACLE_AVOIDANCE, RobotState.EMERGENCY_STOP,
    ],
    RobotState.OBSTACLE_AVOIDANCE: [
        RobotState.STANDING, RobotState.WALKING, RobotState.TURNING,
        RobotState.EMERGENCY_STOP,
    ],
    RobotState.FOLLOWING: [
        RobotState.STANDING, RobotState.WALKING, RobotState.TURNING,
        RobotState.OBSTACLE_AVOIDANCE, RobotState.EMERGENCY_STOP,
    ],
    RobotState.SITTING: [
        RobotState.STANDING, RobotState.EMERGENCY_STOP,
    ],
    RobotState.LAYING_DOWN: [
        RobotState.STANDING, RobotState.EMERGENCY_STOP,
    ],
    RobotState.MANUAL_CONTROL: [
        RobotState.STANDING, RobotState.WALKING, RobotState.TROTTING,
        RobotState.TURNING, RobotState.SITTING, RobotState.LAYING_DOWN,
        RobotState.EMERGENCY_STOP,
    ],
    RobotState.EMERGENCY_STOP: [
        RobotState.IDLE, RobotState.STANDING,
    ],
}


class StateMachine:
    """Robot behavior state machine with transition validation."""

    def __init__(self):
        self.state = RobotState.IDLE
        self._prev_state = RobotState.IDLE
        self._state_start_time = time.time()
        self._history: List[Dict] = []
        self._callbacks: Dict[RobotState, List[Callable]] = {}

        logger.info(f"State machine initialized -> {self.state.value}")

    # ── State Transitions ───────────────────────────────────────

    def transition(self, new_state: RobotState, reason: str = "") -> bool:
        """
        Attempt to transition to a new state.
        Returns True if transition was valid and executed.
        """
        if new_state == self.state:
            return True  # Already in this state

        # Emergency stop always allowed
        if new_state == RobotState.EMERGENCY_STOP:
            self._do_transition(new_state, reason or "EMERGENCY")
            return True

        # Check valid transitions
        allowed = TRANSITIONS.get(self.state, [])
        if new_state not in allowed:
            logger.warning(
                f"Invalid transition: {self.state.value} -> {new_state.value}"
            )
            return False

        self._do_transition(new_state, reason)
        return True

    def _do_transition(self, new_state: RobotState, reason: str):
        self._prev_state = self.state
        duration = time.time() - self._state_start_time

        self._history.append({
            "from": self._prev_state.value,
            "to": new_state.value,
            "reason": reason,
            "duration": round(duration, 2),
            "time": time.time(),
        })
        if len(self._history) > 50:
            self._history.pop(0)

        self.state = new_state
        self._state_start_time = time.time()

        logger.info(
            f"State: {self._prev_state.value} -> {new_state.value}"
            f" ({reason})"
        )

        # Fire callbacks
        for cb in self._callbacks.get(new_state, []):
            try:
                cb(new_state, self._prev_state)
            except Exception as e:
                logger.error(f"Callback error: {e}")

    # ── Action → State Mapping ──────────────────────────────────

    def action_to_state(self, action: str) -> Optional[RobotState]:
        """Map a decision engine action to a robot state."""
        mapping = {
            "walk_forward": RobotState.WALKING,
            "walk": RobotState.WALKING,
            "trot": RobotState.TROTTING,
            "run": RobotState.TROTTING,
            "turn_left": RobotState.TURNING,
            "turn_right": RobotState.TURNING,
            "side_left": RobotState.WALKING,
            "side_right": RobotState.WALKING,
            "stop": RobotState.STANDING,
            "stand": RobotState.STANDING,
            "sit": RobotState.SITTING,
            "lay_down": RobotState.LAYING_DOWN,
            "explore": RobotState.EXPLORING,
            "manual_control": RobotState.MANUAL_CONTROL,
            "avoid_obstacle": RobotState.OBSTACLE_AVOIDANCE,
        }
        return mapping.get(action)

    def apply_action(self, action: str, reason: str = "") -> bool:
        """Convenience: map action name to state and transition."""
        target = self.action_to_state(action)
        if target:
            return self.transition(target, reason or action)
        logger.warning(f"No state mapping for action: {action}")
        return False

    # ── Callbacks ───────────────────────────────────────────────

    def on_enter(self, state: RobotState, callback: Callable):
        """Register a callback for when a state is entered."""
        self._callbacks.setdefault(state, []).append(callback)

    # ── Queries ─────────────────────────────────────────────────

    @property
    def is_moving(self) -> bool:
        return self.state in (
            RobotState.WALKING, RobotState.TROTTING,
            RobotState.TURNING, RobotState.EXPLORING,
        )

    @property
    def is_emergency(self) -> bool:
        return self.state == RobotState.EMERGENCY_STOP

    @property
    def state_duration(self) -> float:
        return time.time() - self._state_start_time

    def emergency_stop(self, reason: str = "safety"):
        self.transition(RobotState.EMERGENCY_STOP, reason)

    def recover_from_emergency(self):
        if self.state == RobotState.EMERGENCY_STOP:
            self.transition(RobotState.STANDING, "recovery")

    # ── Telemetry ───────────────────────────────────────────────

    def get_telemetry(self) -> dict:
        return {
            "state": self.state.value,
            "previous": self._prev_state.value,
            "duration": round(self.state_duration, 1),
            "is_moving": self.is_moving,
            "is_emergency": self.is_emergency,
            "history_len": len(self._history),
            "last_transitions": self._history[-3:] if self._history else [],
        }

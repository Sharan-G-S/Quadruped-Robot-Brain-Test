"""
test_state_machine.py — Unit Tests for Robot State Machine
============================================================
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from brain.state_machine import StateMachine, RobotState


class TestStateMachine:
    """Test state transitions and edge cases."""

    def test_initial_state(self):
        sm = StateMachine()
        assert sm.state == RobotState.IDLE

    def test_valid_transition(self):
        sm = StateMachine()
        assert sm.transition(RobotState.STANDING, "test") is True
        assert sm.state == RobotState.STANDING

    def test_invalid_transition(self):
        sm = StateMachine()
        # IDLE → WALKING is not valid (must go through STANDING)
        assert sm.transition(RobotState.WALKING, "test") is False
        assert sm.state == RobotState.IDLE

    def test_walk_sequence(self):
        sm = StateMachine()
        assert sm.transition(RobotState.STANDING, "boot")
        assert sm.transition(RobotState.WALKING, "walk")
        assert sm.state == RobotState.WALKING

    def test_emergency_stop_from_any_state(self):
        sm = StateMachine()
        sm.transition(RobotState.STANDING, "boot")
        sm.transition(RobotState.WALKING, "walk")
        # Emergency should always work
        sm.emergency_stop("test")
        assert sm.state == RobotState.EMERGENCY_STOP

    def test_recovery_from_emergency(self):
        sm = StateMachine()
        sm.transition(RobotState.STANDING, "boot")
        sm.emergency_stop("test")
        assert sm.state == RobotState.EMERGENCY_STOP
        sm.recover_from_emergency()
        assert sm.state == RobotState.STANDING

    def test_same_state_transition(self):
        sm = StateMachine()
        sm.transition(RobotState.STANDING, "boot")
        # Same state should return True (no-op)
        assert sm.transition(RobotState.STANDING, "again") is True

    def test_apply_action(self):
        sm = StateMachine()
        sm.transition(RobotState.STANDING, "boot")
        assert sm.apply_action("walk_forward", "test")
        assert sm.state == RobotState.WALKING

    def test_is_moving(self):
        sm = StateMachine()
        sm.transition(RobotState.STANDING, "boot")
        assert sm.is_moving is False
        sm.transition(RobotState.WALKING, "walk")
        assert sm.is_moving is True

    def test_telemetry(self):
        sm = StateMachine()
        tel = sm.get_telemetry()
        assert "state" in tel
        assert "is_moving" in tel
        assert "is_emergency" in tel

    def test_sit_requires_stand_to_walk(self):
        sm = StateMachine()
        sm.transition(RobotState.STANDING, "boot")
        sm.transition(RobotState.SITTING, "sit")
        # Can't walk directly from sitting
        assert sm.transition(RobotState.WALKING, "walk") is False
        # Must stand first
        assert sm.transition(RobotState.STANDING, "stand")
        assert sm.transition(RobotState.WALKING, "walk")

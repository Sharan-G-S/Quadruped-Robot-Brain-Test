"""
test_mode_manager.py -- Unit Tests for Control Mode Manager
==============================================================
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from control.mode_manager import ModeManager, ControlMode


class TestModeManager:
    """Test control mode switching and queries."""

    def test_default_autonomous(self):
        mm = ModeManager({})
        assert mm.mode == ControlMode.AUTONOMOUS
        assert mm.is_autonomous is True
        assert mm.is_manual is False

    def test_default_manual(self):
        mm = ModeManager({"default_mode": "manual"})
        assert mm.mode == ControlMode.MANUAL
        assert mm.is_manual is True

    def test_default_hybrid(self):
        mm = ModeManager({"default_mode": "hybrid"})
        assert mm.mode == ControlMode.HYBRID
        assert mm.is_hybrid is True

    def test_switch_to_manual(self):
        mm = ModeManager({})
        assert mm.switch_mode("manual") is True
        assert mm.mode == ControlMode.MANUAL
        assert mm.is_manual is True

    def test_switch_to_hybrid(self):
        mm = ModeManager({})
        assert mm.switch_mode("hybrid") is True
        assert mm.mode == ControlMode.HYBRID
        assert mm.is_hybrid is True
        assert mm.should_run_autonomy is True
        assert mm.should_accept_manual is True

    def test_switch_same_mode(self):
        mm = ModeManager({})
        assert mm.switch_mode("autonomous") is True
        assert mm.mode == ControlMode.AUTONOMOUS

    def test_invalid_mode(self):
        mm = ModeManager({})
        assert mm.switch_mode("invalid_mode") is False
        assert mm.mode == ControlMode.AUTONOMOUS

    def test_switch_disabled(self):
        mm = ModeManager({"allow_mode_switch": False})
        assert mm.switch_mode("manual") is False
        assert mm.mode == ControlMode.AUTONOMOUS

    def test_should_run_autonomy(self):
        mm = ModeManager({"default_mode": "autonomous"})
        assert mm.should_run_autonomy is True
        mm.switch_mode("manual")
        assert mm.should_run_autonomy is False
        mm.switch_mode("hybrid")
        assert mm.should_run_autonomy is True

    def test_should_accept_manual(self):
        mm = ModeManager({"default_mode": "autonomous"})
        assert mm.should_accept_manual is False
        mm.switch_mode("manual")
        assert mm.should_accept_manual is True
        mm.switch_mode("hybrid")
        assert mm.should_accept_manual is True

    def test_telemetry(self):
        mm = ModeManager({})
        tel = mm.get_telemetry()
        assert "mode" in tel
        assert tel["mode"] == "autonomous"
        assert "switch_count" in tel
        assert "allow_switch" in tel

    def test_switch_count(self):
        mm = ModeManager({})
        mm.switch_mode("manual")
        mm.switch_mode("hybrid")
        mm.switch_mode("autonomous")
        tel = mm.get_telemetry()
        assert tel["switch_count"] == 3

    def test_previous_mode_tracked(self):
        mm = ModeManager({})
        mm.switch_mode("manual")
        tel = mm.get_telemetry()
        assert tel["previous_mode"] == "autonomous"

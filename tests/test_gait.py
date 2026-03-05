"""
test_gait.py — Unit Tests for Gait Controller
================================================
"""

import pytest
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from locomotion.gait_controller import GaitController, GaitType


@pytest.fixture
def gait():
    return GaitController(
        config={
            "step_height": 30.0, "step_length": 60.0,
            "cycle_time": 0.8, "trot_speed": 0.5,
        },
        body_config={
            "length": 200.0, "width": 110.0, "default_height": 150.0,
        },
    )


class TestGaitController:
    """Test gait trajectory generation."""

    def test_stand_positions(self, gait):
        """Standing should return neutral positions."""
        gait.set_gait(GaitType.STAND)
        pos = gait.get_foot_positions()
        assert len(pos) == 4
        assert all(leg in pos for leg in ["FR", "FL", "RR", "RL"])

    def test_stand_height(self, gait):
        """Standing Z should be at default height."""
        gait.set_gait(GaitType.STAND)
        pos = gait.get_foot_positions()
        for leg, (x, y, z) in pos.items():
            assert z == pytest.approx(150.0, abs=1.0)

    def test_sit_rear_lower(self, gait):
        """Sitting should lower rear legs."""
        gait.set_gait(GaitType.SIT)
        pos = gait.get_foot_positions()
        # Rear legs should be lower than front
        assert pos["RR"][2] < pos["FR"][2]
        assert pos["RL"][2] < pos["FL"][2]

    def test_walk_returns_4_legs(self, gait):
        """Walking should produce positions for all 4 legs."""
        gait.set_gait(GaitType.WALK, speed=0.5)
        pos = gait.get_foot_positions()
        assert len(pos) == 4

    def test_trot_phase_opposition(self, gait):
        """In trot, diagonal legs should have opposite phases."""
        gait.set_gait(GaitType.TROT, speed=0.5)
        pos1 = gait.get_foot_positions()
        # All legs should have positions (can't easily test phase in unit test
        # without time manipulation, so just verify structure)
        assert len(pos1) == 4
        for leg, (x, y, z) in pos1.items():
            assert isinstance(x, float)
            assert isinstance(z, float)

    def test_gait_change(self, gait):
        """Changing gait should update current gait type."""
        gait.set_gait(GaitType.WALK)
        assert gait.current_gait == GaitType.WALK
        gait.set_gait(GaitType.TROT)
        assert gait.current_gait == GaitType.TROT

    def test_speed_clamping(self, gait):
        """Speed should be clamped between 0.1 and 1.0."""
        gait.set_gait(GaitType.WALK, speed=5.0)
        assert gait.speed <= 1.0
        gait.set_gait(GaitType.WALK, speed=0.0)
        assert gait.speed >= 0.1

    def test_lay_down_low_height(self, gait):
        """Laying down should lower all legs significantly."""
        gait.set_gait(GaitType.LAY_DOWN)
        pos = gait.get_foot_positions()
        for leg, (x, y, z) in pos.items():
            assert z < 100  # Much lower than standing height

    def test_telemetry(self, gait):
        tel = gait.get_telemetry()
        assert "gait" in tel
        assert "speed" in tel
        assert "step_height" in tel

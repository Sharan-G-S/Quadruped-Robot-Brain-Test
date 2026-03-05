"""
test_kinematics.py — Unit Tests for Inverse Kinematics
========================================================
"""

import pytest
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from locomotion.kinematics import InverseKinematics


@pytest.fixture
def ik():
    return InverseKinematics({
        "hip_length": 50.0,
        "upper_length": 107.0,
        "lower_length": 130.0,
    })


class TestInverseKinematics:
    """Test IK solver for correctness and edge cases."""

    def test_solve_standing_position(self, ik):
        """Standing position should return valid angles."""
        result = ik.solve(0, 50, 150)
        assert result is not None
        hip, upper, lower = result
        assert 0 <= hip <= 180
        assert 0 <= upper <= 180
        assert 0 <= lower <= 180

    def test_solve_center_position(self, ik):
        """Position directly below hip should have hip ≈ 90°."""
        result = ik.solve(0, 50, 130)
        assert result is not None
        hip, _, _ = result
        assert 80 <= hip <= 100  # Should be close to 90

    def test_solve_forward_reach(self, ik):
        """Forward position should be reachable."""
        result = ik.solve(60, 50, 140)
        assert result is not None
        hip, upper, lower = result
        assert 0 <= hip <= 180
        assert 0 <= upper <= 180
        assert 0 <= lower <= 180

    def test_unreachable_position_clamped(self, ik):
        """Very distant position should still return clamped angles."""
        result = ik.solve(0, 50, 500)
        # Should return clamped result, not None
        assert result is not None

    def test_solve_mirrored(self, ik):
        """Mirrored solve should produce different hip angle."""
        right = ik.solve(0, 50, 150, mirror=False)
        left = ik.solve(0, 50, 150, mirror=True)
        assert right is not None and left is not None
        # Hip angles should differ (mirrored)
        assert abs(right[0] - left[0]) > 1.0

    def test_forward_kinematics_roundtrip(self, ik):
        """FK(IK(x,y,z)) should approximate (x,y,z)."""
        target = (30, 50, 140)
        angles = ik.solve(*target)
        if angles:
            result = ik.forward(*angles)
            # Allow tolerance of ~10mm due to numerical precision
            assert abs(result[0] - target[0]) < 15
            assert abs(result[2] - target[2]) < 15

    def test_solve_all_legs(self, ik):
        """Batch solve for all 4 legs should return valid results."""
        positions = {
            "FR": (50, 55, 150),
            "FL": (50, -55, 150),
            "RR": (-50, 55, 150),
            "RL": (-50, -55, 150),
        }
        result = ik.solve_all_legs(positions)
        assert len(result) == 4
        for leg, angles in result.items():
            assert len(angles) == 3
            for angle in angles:
                assert 0 <= angle <= 180

    def test_workspace_check(self, ik):
        """Workspace check should distinguish reachable from unreachable."""
        assert ik.workspace_check(0, 50, 150) is True
        # Very far away should be unreachable
        assert ik.workspace_check(0, 50, 500) is False

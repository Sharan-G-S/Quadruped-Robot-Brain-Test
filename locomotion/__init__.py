# QuadBot-AI — Locomotion Engine
from .kinematics import InverseKinematics
from .gait_controller import GaitController
from .body_controller import BodyController

__all__ = ["InverseKinematics", "GaitController", "BodyController"]

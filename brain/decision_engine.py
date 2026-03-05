"""
decision_engine.py — LLM Scene Analysis → Robot Action Mapping
================================================================
The "thinking" center: takes sensor data + vision results and decides
what the robot should do next.
"""

import logging
import time
from typing import Optional, Dict, Any, List

from brain.llm_client import LLMClient

logger = logging.getLogger(__name__)

DECISION_SYSTEM_PROMPT = """You are the decision-making brain of a quadruped robot dog called QuadBot-AI.
You receive sensor data and scene analysis, then decide the robot's next action.

Available actions:
  walk_forward  — Walk straight ahead
  trot          — Move faster
  turn_left     — Turn left
  turn_right    — Turn right
  side_left     — Side-step left
  side_right    — Side-step right
  stop          — Stop and stand
  sit           — Sit down
  lay_down      — Lay down
  explore       — Look around for interesting things

Respond with a JSON object:
{
  "action": "action_name",
  "speed": 0.1-1.0,
  "duration": seconds (0 = continuous),
  "reasoning": "brief explanation of why this action",
  "priority": "low|medium|high|critical"
}

Rules:
1. Safety first — avoid obstacles and hazards
2. If obstacle is near and center, stop or turn
3. If path is clear, prefer walking/trotting
4. If asked to go somewhere, navigate toward that target
5. Be decisive — pick one action, not a sequence"""


class DecisionEngine:
    """Maps LLM analysis and sensor data to robot actions."""

    def __init__(self, llm_client: LLMClient):
        self.llm = llm_client
        self._history: List[Dict] = []
        self._max_history = 10
        self._last_decision: Optional[Dict] = None

    def decide(self,
               vision_result: Optional[Dict] = None,
               sensor_data: Optional[Dict] = None,
               user_command: Optional[str] = None,
               current_state: str = "idle") -> Dict[str, Any]:
        """
        Make a decision about the robot's next action.

        Priority order:
          1. Direct user commands (highest)
          2. Safety overrides (obstacle too close)
          3. LLM vision-based decisions
          4. Default behavior (stand/explore)
        """
        # Priority 1: Safety override — immediate obstacle
        if sensor_data and sensor_data.get("obstacle_detected"):
            distance = sensor_data.get("distance_cm", 100)
            decision = {
                "action": "stop" if distance < 15 else "turn_right",
                "speed": 0.3,
                "duration": 0,
                "reasoning": f"Obstacle at {distance:.0f}cm — emergency avoidance",
                "priority": "critical",
            }
            self._record(decision)
            return decision

        # Priority 2: Direct user command
        if user_command:
            return self._process_user_command(user_command, vision_result)

        # Priority 3: Vision-based LLM decision
        if vision_result:
            return self._vision_decision(vision_result, sensor_data, current_state)

        # Priority 4: Default — stay idle
        return self._default_action(current_state)

    def _process_user_command(self, command: str,
                               vision: Optional[Dict]) -> Dict[str, Any]:
        """Process a natural language user command through LLM."""
        prompt = f"""The user commands the robot: "{command}"

Current vision analysis: {vision if vision else 'no data'}

What action should the robot take?"""

        result = self.llm.query_json(prompt, DECISION_SYSTEM_PROMPT)

        if result and "action" in result:
            self._record(result)
            return result

        # Fallback: simple keyword matching
        return self._keyword_match(command)

    def _vision_decision(self, vision: Dict, sensors: Optional[Dict],
                          state: str) -> Dict[str, Any]:
        """Make a decision based on vision analysis."""
        # Quick path: use the vision's own suggested action
        suggested = vision.get("suggested_action", "stop")
        path_clear = vision.get("path_clear", True)

        if not path_clear:
            obstacles = [o for o in vision.get("objects", [])
                        if o.get("is_obstacle")]
            if obstacles:
                pos = obstacles[0].get("position", "center")
                if pos == "left":
                    action = "turn_right"
                elif pos == "right":
                    action = "turn_left"
                else:
                    action = "turn_right"  # Default avoidance direction

                decision = {
                    "action": action,
                    "speed": 0.3,
                    "duration": 0,
                    "reasoning": f"Obstacle {pos}, turning to avoid",
                    "priority": "high",
                }
                self._record(decision)
                return decision

        # Path is clear — follow suggested action
        decision = {
            "action": suggested if suggested != "explore" else "walk_forward",
            "speed": 0.5 if path_clear else 0.3,
            "duration": 0,
            "reasoning": vision.get("scene_description", "Following vision guidance"),
            "priority": "medium",
        }
        self._record(decision)
        return decision

    def _keyword_match(self, command: str) -> Dict[str, Any]:
        """Simple keyword-based command matching as fallback."""
        cmd = command.lower().strip()
        mappings = [
            (["walk", "go", "forward", "move"], "walk_forward", 0.4),
            (["run", "fast", "trot"], "trot", 0.7),
            (["left", "turn left"], "turn_left", 0.4),
            (["right", "turn right"], "turn_right", 0.4),
            (["stop", "halt", "stay", "freeze"], "stop", 0),
            (["sit", "sit down"], "sit", 0),
            (["down", "lay", "lie"], "lay_down", 0),
            (["stand", "up", "get up"], "stand", 0),
        ]

        for keywords, action, speed in mappings:
            if any(kw in cmd for kw in keywords):
                decision = {
                    "action": action,
                    "speed": speed,
                    "duration": 0,
                    "reasoning": f"Keyword match: '{command}'",
                    "priority": "medium",
                }
                self._record(decision)
                return decision

        decision = {
            "action": "stop",
            "speed": 0,
            "duration": 0,
            "reasoning": f"Unrecognized command: '{command}'",
            "priority": "low",
        }
        self._record(decision)
        return decision

    def _default_action(self, state: str) -> Dict[str, Any]:
        return {
            "action": "stop",
            "speed": 0,
            "duration": 0,
            "reasoning": f"No input, current state: {state}",
            "priority": "low",
        }

    def _record(self, decision: Dict):
        self._last_decision = decision
        self._history.append({
            "time": time.time(),
            "decision": decision,
        })
        if len(self._history) > self._max_history:
            self._history.pop(0)
        logger.info(f"🧠 Decision: {decision['action']} — {decision['reasoning']}")

    @property
    def last_decision(self) -> Optional[Dict]:
        return self._last_decision

    def get_telemetry(self) -> dict:
        return {
            "last_action": self._last_decision.get("action") if self._last_decision else None,
            "last_reasoning": self._last_decision.get("reasoning") if self._last_decision else None,
            "decision_count": len(self._history),
        }

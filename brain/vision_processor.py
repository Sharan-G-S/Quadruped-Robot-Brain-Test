"""
vision_processor.py — Camera Frame → LLM Scene Analysis
=========================================================
Captures camera frames, sends them to the LLM for visual understanding,
and returns structured scene descriptions.
"""

import logging
import time
from typing import Optional, Dict, Any

from brain.llm_client import LLMClient

logger = logging.getLogger(__name__)

# System prompt that tells the LLM it is the eyes of a robot dog
VISION_SYSTEM_PROMPT = """You are the visual perception system of a quadruped robot dog called QuadBot-AI.
You receive camera images from the robot's front-facing camera and must analyze the scene.

Your job is to return a JSON object describing what you see:

{
  "scene_description": "Brief description of the environment",
  "objects": [
    {
      "name": "object name",
      "position": "left|center|right",
      "distance": "near|medium|far",
      "is_obstacle": true/false,
      "is_interesting": true/false
    }
  ],
  "floor_type": "indoor|outdoor|grass|concrete|unknown",
  "path_clear": true/false,
  "suggested_action": "walk_forward|turn_left|turn_right|stop|explore|avoid_obstacle",
  "confidence": 0.0-1.0,
  "hazards": ["list of any dangers detected"]
}

Be concise and practical. Focus on navigation-relevant information.
The robot has 4 legs and can walk, turn, side-step, sit, and lay down.
Prioritize safety — if you see obstacles or hazards, suggest avoidance."""


class VisionProcessor:
    """Processes camera frames through LLM for scene understanding."""

    def __init__(self, llm_client: LLMClient, config: dict):
        self.llm = llm_client
        self.interval = config.get("vision_interval_sec", 2.0)
        self._last_analysis_time = 0
        self._last_result: Optional[Dict] = None
        self._analysis_count = 0

    def analyze_frame(self, image_base64: str,
                      context: str = "") -> Optional[Dict[str, Any]]:
        """
        Send a camera frame to the LLM for scene analysis.

        Args:
            image_base64: JPEG frame encoded as base64 string
            context: Additional context (e.g. current state, mission)

        Returns:
            Parsed JSON scene analysis or None on failure.
        """
        prompt = "Analyze this camera frame from the robot dog."
        if context:
            prompt += f"\n\nCurrent context: {context}"

        result = self.llm.query_json(
            prompt=prompt,
            system_prompt=VISION_SYSTEM_PROMPT,
            image_base64=image_base64,
        )

        if result:
            self._last_result = result
            self._analysis_count += 1
            self._last_analysis_time = time.time()
            logger.info(f"👁️ Scene: {result.get('scene_description', 'N/A')}")
        else:
            logger.warning("Vision analysis returned no result")

        return result

    def should_analyze(self) -> bool:
        """Check if enough time has passed for a new analysis."""
        return (time.time() - self._last_analysis_time) >= self.interval

    def get_suggested_action(self) -> Optional[str]:
        """Get the last suggested action from vision analysis."""
        if self._last_result:
            return self._last_result.get("suggested_action")
        return None

    def is_path_clear(self) -> bool:
        """Check if the path ahead is clear based on vision."""
        if self._last_result:
            return self._last_result.get("path_clear", True)
        return True  # Default: assume clear

    def get_obstacles(self) -> list:
        """Return list of detected obstacles."""
        if not self._last_result:
            return []
        objects = self._last_result.get("objects", [])
        return [o for o in objects if o.get("is_obstacle")]

    @property
    def last_result(self) -> Optional[Dict]:
        return self._last_result

    def get_telemetry(self) -> dict:
        return {
            "analyses_count": self._analysis_count,
            "last_analysis_age": round(time.time() - self._last_analysis_time, 1)
            if self._last_analysis_time > 0 else None,
            "path_clear": self.is_path_clear(),
            "suggested_action": self.get_suggested_action(),
            "scene": self._last_result.get("scene_description") if self._last_result else None,
        }

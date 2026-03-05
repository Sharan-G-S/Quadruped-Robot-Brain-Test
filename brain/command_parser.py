"""
command_parser.py — Natural Language Command Parser
====================================================
Parses text/voice commands into structured robot actions using the LLM.
"""

import logging
from typing import Optional, Dict

from brain.llm_client import LLMClient

logger = logging.getLogger(__name__)

PARSER_SYSTEM_PROMPT = """You are a command parser for a robot dog called QuadBot-AI.
Convert natural language commands into structured JSON.

Output format:
{
  "intent": "move|turn|pose|navigate|query|unknown",
  "action": "walk_forward|trot|turn_left|turn_right|side_left|side_right|stop|sit|stand|lay_down",
  "parameters": {
    "speed": 0.1-1.0,
    "duration": seconds or 0 for continuous,
    "target": "optional target object or direction",
    "distance": "optional distance in steps"
  },
  "confidence": 0.0-1.0
}

Examples:
"walk forward slowly" → {"intent":"move", "action":"walk_forward", "parameters":{"speed":0.3}, "confidence":0.95}
"go to the red ball" → {"intent":"navigate", "action":"walk_forward", "parameters":{"target":"red ball"}, "confidence":0.8}
"sit down boy" → {"intent":"pose", "action":"sit", "parameters":{}, "confidence":0.95}
"what do you see" → {"intent":"query", "action":"stop", "parameters":{"target":"describe_scene"}, "confidence":0.9}"""


class CommandParser:
    """Parses natural language commands into structured robot actions."""

    def __init__(self, llm_client: LLMClient):
        self.llm = llm_client

    def parse(self, command: str) -> Dict:
        """
        Parse a natural language command into a structured action.

        Args:
            command: Natural language string (text or voice transcription)

        Returns:
            Structured command dict with intent, action, parameters.
        """
        if not command or not command.strip():
            return {"intent": "unknown", "action": "stop",
                    "parameters": {}, "confidence": 0.0}

        # Try LLM parsing first
        result = self.llm.query_json(
            prompt=f'Parse this robot command: "{command}"',
            system_prompt=PARSER_SYSTEM_PROMPT,
        )

        if result and result.get("confidence", 0) > 0.3:
            logger.info(f"🗣️ Parsed: '{command}' → {result.get('action')}")
            return result

        # Fallback: keyword-based parsing
        return self._keyword_parse(command)

    def _keyword_parse(self, command: str) -> Dict:
        """Fallback keyword-based command parsing."""
        cmd = command.lower().strip()

        patterns = [
            (["walk", "go forward", "move forward", "advance"],
             "move", "walk_forward", {"speed": 0.4}),
            (["run", "fast", "trot", "quick"],
             "move", "trot", {"speed": 0.7}),
            (["turn left", "go left", "left"],
             "turn", "turn_left", {"speed": 0.4}),
            (["turn right", "go right", "right"],
             "turn", "turn_right", {"speed": 0.4}),
            (["stop", "halt", "freeze", "wait", "stay"],
             "pose", "stop", {}),
            (["sit", "sit down"],
             "pose", "sit", {}),
            (["stand", "get up", "up"],
             "pose", "stand", {}),
            (["lay", "lie", "down", "rest"],
             "pose", "lay_down", {}),
            (["look", "see", "what", "where", "describe"],
             "query", "stop", {"target": "describe_scene"}),
            (["come", "here", "follow"],
             "navigate", "walk_forward", {"target": "user"}),
            (["back", "reverse", "backward"],
             "move", "walk_forward", {"speed": -0.3}),
        ]

        for keywords, intent, action, params in patterns:
            if any(kw in cmd for kw in keywords):
                result = {
                    "intent": intent, "action": action,
                    "parameters": params, "confidence": 0.7,
                }
                logger.info(f"🗣️ Keyword parsed: '{command}' → {action}")
                return result

        return {
            "intent": "unknown", "action": "stop",
            "parameters": {}, "confidence": 0.1,
        }

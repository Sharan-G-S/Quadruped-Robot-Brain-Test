"""
llm_client.py — LLM API Abstraction (OpenAI + Google Gemini)
=============================================================
Unified interface for sending prompts (text + images) to GPT-4o or Gemini.
"""

import logging
import time
import json
from typing import Optional, List, Dict, Any

logger = logging.getLogger(__name__)


class LLMClient:
    """Unified LLM client supporting OpenAI and Google Gemini."""

    def __init__(self, config: dict):
        self.provider = config.get("provider", "openai")
        self.temperature = config.get("temperature", 0.3)
        self._client = None
        self._model = None

        if self.provider == "openai":
            self._init_openai(config.get("openai", {}))
        elif self.provider == "gemini":
            self._init_gemini(config.get("gemini", {}))
        else:
            logger.error(f"Unknown LLM provider: {self.provider}")

    def _init_openai(self, cfg: dict):
        try:
            import openai
            api_key = cfg.get("api_key", "")
            if not api_key or api_key.startswith("YOUR_"):
                logger.warning("⚠️  OpenAI API key not set — LLM calls will fail")
            self._client = openai.OpenAI(api_key=api_key)
            self._model = cfg.get("model", "gpt-4o")
            self._max_tokens = cfg.get("max_tokens", 1024)
            logger.info(f"✅ OpenAI client: {self._model}")
        except ImportError:
            logger.error("openai package not installed")
        except Exception as e:
            logger.error(f"OpenAI init error: {e}")

    def _init_gemini(self, cfg: dict):
        try:
            import google.generativeai as genai
            api_key = cfg.get("api_key", "")
            if not api_key or api_key.startswith("YOUR_"):
                logger.warning("⚠️  Gemini API key not set — LLM calls will fail")
            genai.configure(api_key=api_key)
            self._model = cfg.get("model", "gemini-2.0-flash")
            self._max_tokens = cfg.get("max_tokens", 1024)
            self._client = genai.GenerativeModel(self._model)
            logger.info(f"✅ Gemini client: {self._model}")
        except ImportError:
            logger.error("google-generativeai package not installed")
        except Exception as e:
            logger.error(f"Gemini init error: {e}")

    # ── Text Query ──────────────────────────────────────────────

    def query(self, prompt: str, system_prompt: str = "") -> Optional[str]:
        """Send a text-only prompt and return the response string."""
        try:
            if self.provider == "openai":
                return self._openai_text(prompt, system_prompt)
            elif self.provider == "gemini":
                return self._gemini_text(prompt, system_prompt)
        except Exception as e:
            logger.error(f"LLM query error: {e}")
            return None

    # ── Vision Query (Text + Image) ─────────────────────────────

    def query_with_image(self, prompt: str, image_base64: str,
                         system_prompt: str = "") -> Optional[str]:
        """Send text + base64 image and return the response."""
        try:
            if self.provider == "openai":
                return self._openai_vision(prompt, image_base64, system_prompt)
            elif self.provider == "gemini":
                return self._gemini_vision(prompt, image_base64, system_prompt)
        except Exception as e:
            logger.error(f"LLM vision error: {e}")
            return None

    # ── JSON Query ──────────────────────────────────────────────

    def query_json(self, prompt: str, system_prompt: str = "",
                   image_base64: Optional[str] = None) -> Optional[dict]:
        """Query LLM and parse response as JSON."""
        full_prompt = prompt + "\n\nRespond ONLY with valid JSON, no markdown."

        if image_base64:
            raw = self.query_with_image(full_prompt, image_base64, system_prompt)
        else:
            raw = self.query(full_prompt, system_prompt)

        if not raw:
            return None

        return self._parse_json(raw)

    # ── OpenAI Implementation ───────────────────────────────────

    def _openai_text(self, prompt: str, system_prompt: str) -> Optional[str]:
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({"role": "user", "content": prompt})

        response = self._client.chat.completions.create(
            model=self._model,
            messages=messages,
            max_tokens=self._max_tokens,
            temperature=self.temperature,
        )
        return response.choices[0].message.content

    def _openai_vision(self, prompt: str, image_b64: str,
                       system_prompt: str) -> Optional[str]:
        messages = []
        if system_prompt:
            messages.append({"role": "system", "content": system_prompt})
        messages.append({
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{image_b64}",
                        "detail": "low",
                    },
                },
            ],
        })

        response = self._client.chat.completions.create(
            model=self._model,
            messages=messages,
            max_tokens=self._max_tokens,
            temperature=self.temperature,
        )
        return response.choices[0].message.content

    # ── Gemini Implementation ───────────────────────────────────

    def _gemini_text(self, prompt: str, system_prompt: str) -> Optional[str]:
        full = f"{system_prompt}\n\n{prompt}" if system_prompt else prompt
        response = self._client.generate_content(
            full,
            generation_config={"max_output_tokens": self._max_tokens,
                               "temperature": self.temperature},
        )
        return response.text

    def _gemini_vision(self, prompt: str, image_b64: str,
                       system_prompt: str) -> Optional[str]:
        import base64
        image_bytes = base64.b64decode(image_b64)
        image_part = {"mime_type": "image/jpeg", "data": image_bytes}
        full = f"{system_prompt}\n\n{prompt}" if system_prompt else prompt

        response = self._client.generate_content(
            [full, image_part],
            generation_config={"max_output_tokens": self._max_tokens,
                               "temperature": self.temperature},
        )
        return response.text

    # ── Helpers ─────────────────────────────────────────────────

    @staticmethod
    def _parse_json(raw: str) -> Optional[dict]:
        """Extract JSON from LLM response (handles markdown code blocks)."""
        text = raw.strip()
        # Strip markdown code blocks
        if text.startswith("```"):
            lines = text.split("\n")
            lines = [l for l in lines if not l.strip().startswith("```")]
            text = "\n".join(lines)
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            # Try to find JSON object in the response
            start = text.find("{")
            end = text.rfind("}") + 1
            if start >= 0 and end > start:
                try:
                    return json.loads(text[start:end])
                except json.JSONDecodeError:
                    pass
            logger.warning(f"Could not parse JSON from LLM: {text[:200]}")
            return None

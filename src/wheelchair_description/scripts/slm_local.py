"""Local SLM adapter that calls a local HTTP chat/completions server (llama.cpp or similar).

Defaults to SLM_LOCAL_URL http://127.0.0.1:8092/v1/chat/completions. Sends a strict system prompt
asking the model to return ONLY a JSON object: {intent, target, target_type, confidence} and
returns a parsed Python dict. If the server is unreachable or returns unparsable output, a small
rule-based fallback parser is used so simple "go to X" utterances still work.

Usage:
    from slm_local import SLMLocal
    slm = SLMLocal()
    res = slm.parse_command("take me to the stove")

No secrets or API keys are stored here; the server is local.
"""

from typing import Optional, Dict, Any
import os
import json
import re
import requests
import logging

logger = logging.getLogger(__name__)

DEFAULT_URL = os.environ.get("SLM_LOCAL_URL", "http://127.0.0.1:8092/v1/chat/completions")
TIMEOUT = int(os.environ.get("SLM_LOCAL_TIMEOUT", "6"))

# Small rule-based fallback parser
GO_RE = re.compile(r"\b(?:go|take me|bring me|drive|move|head|navigate)\b.*?\b(?:to|towards?)\s+(?:the |a |an |my )?(?P<t>[a-z ]+)", re.I)
STOP_RE = re.compile(r"\b(stop|halt|cancel)\b", re.I)


class SLMLocal:
    def __init__(self, url: Optional[str] = None, timeout: Optional[int] = None):
        self.url = url or DEFAULT_URL
        self.timeout = timeout or TIMEOUT
        self.system_prompt = (
            "You are a navigation language model. For the single user utterance provided, "
            "return ONLY a JSON object with keys: intent (go_to|stop|unknown), target (short noun phrase), "
            "target_type (place|object|unknown), and confidence (0.0-1.0). Return nothing else."
        )

    def _call_server(self, text: str) -> Optional[str]:
        # POST a chat/completions-style payload; adapt if your server uses a different schema
        payload = {
            "model": os.environ.get("SLM_LOCAL_MODEL", "qwen2.5-1.5b-instruct"),
            "messages": [
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f'Utterance: "{text}"'}
            ],
            "max_tokens": 128,
            "temperature": 0.0,
        }
        try:
            r = requests.post(self.url, json=payload, timeout=self.timeout)
            r.raise_for_status()
            data = r.json()
            # Try common response structures
            # OpenAI-like: data['choices'][0]['message']['content']
            if isinstance(data, dict):
                choices = data.get("choices") or data.get("results")
                if choices and isinstance(choices, list):
                    first = choices[0]
                    if isinstance(first, dict):
                        # message.content
                        if "message" in first and isinstance(first["message"], dict):
                            return first["message"].get("content")
                        if "content" in first:
                            return first.get("content")
                        if "text" in first:
                            return first.get("text")
                # some servers return top-level 'content' or 'output'
                if "content" in data:
                    return data.get("content")
                if "output" in data:
                    return data.get("output")
            # otherwise return raw text
            return r.text
        except Exception as e:
            logger.warning("SLM local call failed: %s", e)
            return None

    def _extract_json(self, s: str) -> Optional[Dict[str, Any]]:
        if not s:
            return None
        # Try direct JSON
        try:
            return json.loads(s)
        except Exception:
            pass
        # Extract first {...} substring
        m = re.search(r"\{.*\}", s, re.S)
        if not m:
            return None
        try:
            return json.loads(m.group(0))
        except Exception:
            return None

    def _fallback(self, text: str) -> Dict[str, Any]:
        t = text.strip().lower()
        if STOP_RE.search(t):
            return {"intent": "stop", "target": "", "target_type": "unknown", "confidence": 1.0}
        m = GO_RE.search(t)
        if m:
            target = m.group("t").strip().rstrip(".")
            return {"intent": "go_to", "target": target, "target_type": "place", "confidence": 0.6}
        bare = re.sub(r"[^a-z ]", "", t).strip()
        if 1 <= len(bare.split()) <= 4:
            return {"intent": "go_to", "target": bare, "target_type": "place", "confidence": 0.5}
        return {"intent": "unknown", "target": t, "target_type": "unknown", "confidence": 0.0}

    def parse_command(self, text: str) -> Dict[str, Any]:
        if not text:
            return {"intent": "unknown", "target": "", "target_type": "unknown", "confidence": 0.0}
        raw = self._call_server(text)
        if raw:
            parsed = self._extract_json(raw if isinstance(raw, str) else json.dumps(raw))
            if parsed and "intent" in parsed:
                return {
                    "intent": parsed.get("intent", "unknown"),
                    "target": (parsed.get("target") or "").strip(),
                    "target_type": parsed.get("target_type", "unknown"),
                    "confidence": float(parsed.get("confidence", 0.0)),
                    "raw": raw,
                }
        # fallback
        return self._fallback(text)

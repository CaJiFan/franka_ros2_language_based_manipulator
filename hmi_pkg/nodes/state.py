#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
State management module - manages application state
"""

from typing import Set, Optional, Dict
from dataclasses import dataclass, field
from fastapi import WebSocket
import json
import logging

log = logging.getLogger("hmi.state")

@dataclass
class ASRState:
    """Global ASR and application state"""
    sr: int = 16000
    bytes_total: int = 0
    last_rms: float = 0.0
    clients: Set[WebSocket] = field(default_factory=set)
    last_text: str = ""
    last_intent: Optional[Dict] = None
    last_error: str = ""
    utter_seq: int = 0
    current_sig: str = ""
    last_confirmed_sig: str = ""
    last_tx_norm: str = ""
    last_tx_time: float = 0.0
    pending_action: Optional[str] = None
    pending_object: Optional[str] = None
    pending_ts: float = 0.0
    locked: bool = False

# Global state instance
STATE = ASRState()

def sig_utter(seq: int) -> str:
    """Generate utterance signature"""
    return f"u{seq:08d}"

def norm_txt(t: str) -> str:
    """Normalize text for comparison"""
    import re
    t = t.lower()
    t = re.sub(r"[^a-z0-9]+", " ", t).strip()
    return t

async def broadcast(payload: Dict):
    """Broadcast message to all connected WebSocket clients"""
    dead = []
    for c in list(STATE.clients):
        try:
            await c.send_text(json.dumps(payload))
        except:
            dead.append(c)
    for c in dead:
        STATE.clients.discard(c)

def is_confirmable(intent: Dict) -> bool:
    """Check if an intent requires confirmation"""
    act = (intent or {}).get("action")
    obj = (intent or {}).get("object")
    if not act: 
        return False
    if act == "pick":
        return obj is not None
    # one-word actions are confirmable alone
    return True

def reset_pending_intent():
    """Reset pending intent state"""
    STATE.pending_action = None
    STATE.pending_object = None
    STATE.pending_ts = 0.0

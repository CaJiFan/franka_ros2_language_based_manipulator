#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ASR (Automatic Speech Recognition) module using OpenAI Whisper
"""

import os
import re
import tempfile
import traceback
import logging
from typing import Optional, Tuple
from openai import OpenAI
from config import Config
from audio_utils import rms_int16, ms_of_bytes, amplify_pcm16, write_wav, calculate_gain
from nlu_engine import load_capabilities_for_prompt

log = logging.getLogger("hmi.asr")

# Initialize OpenAI client
client = OpenAI(api_key=Config.OPENAI_KEY) if Config.OPENAI_KEY else None

def _whisper_prompt() -> Optional[str]:
    """Generate prompt for Whisper based on capabilities"""
    if not Config.WHISPER_USE_PROMPT:
        return None
    
    actions, objects = load_capabilities_for_prompt()
    if not actions and not objects:
        return None
    
    hint = (
        "Robot control interface. Listen for these commands.\n"
        f"ACTIONS: {', '.join(actions)}\nOBJECTS: {', '.join(objects)}\n"
    )
    return hint

def sanitize_transcript(text: str, prompt_used: Optional[str]) -> str:
    """Remove any prompt artifacts from transcript"""
    if not text: return text
    
    t = text
    if prompt_used: 
        t = t.replace(prompt_used, "")
    
    # Remove prompt-like patterns
    patterns = [
        r"(?i)^Robot control interface.*$",
        r"(?i)^ACTIONS:.*$",
        r"(?i)^OBJECTS:.*$",
    ]
    for pat in patterns:
        t = re.sub(pat, "", t, flags=re.M)
    
    t = re.sub(r"\s{2,}", " ", t).strip()
    return t

def _post_process_transcript(text: str) -> str:
    """Apply post-processing corrections to transcript"""
    if not text: return text
    
    corrections = [
        (r'\bike\s+aken\b', 'pick can'),
        (r'\bpeak\s+can\b', 'pick can'),
        (r'\bpicca\s*bott?o\b', 'pick bottle'),
        (r'\bpick\s+bott?al\b', 'pick bottle'),
        (r'\bcouncil\b', 'cancel'),
        (r'\bcounsel\b', 'cancel'),
        (r'\brel\s+ease\b', 'release'),
    ]
    
    result = text
    for pattern, replacement in corrections:
        result = re.sub(pattern, replacement, result, flags=re.IGNORECASE)
    return result

def _english_like(text: str) -> bool:
    """Check if text appears to be English"""
    if not Config.EN_ONLY_STRICT: return True
    return all(ord(c) < 128 for c in text)

async def transcribe_with_openai(pcm: bytes, sr: int, last_error_ref: list) -> str:
    """
    Transcribe audio using OpenAI Whisper
    last_error_ref: list containing one string element for error reporting
    """
    if not client:
        last_error_ref[0] = "OpenAI key not set"
        return ""
    
    if not pcm:
        return ""
    
    seg_ms = ms_of_bytes(len(pcm), sr)
    if seg_ms < Config.ASR_MIN_MS:
        return ""
    
    seg_rms = rms_int16(pcm)
    if (not Config.WHISPER_BYPASS_RMS) and seg_rms < Config.WHISPER_MIN_RMS:
        last_error_ref[0] = f"Segment RMS {seg_rms:.3f} < {Config.WHISPER_MIN_RMS}"
        return ""
    
    # Amplify if needed
    gain = calculate_gain(seg_rms)
    if gain > 1.0:
        pcm = amplify_pcm16(pcm, gain)
    
    tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
    try:
        write_wav(tmp.name, pcm, sr)
        tmp.close()
        
        prompt = _whisper_prompt()
        kwargs = {
            "model": Config.WHISPER_MODEL, 
            "file": open(tmp.name, "rb"), 
            "language": Config.WHISPER_LANG
        }
        if Config.WHISPER_TEMPERATURE > 0: 
            kwargs["temperature"] = Config.WHISPER_TEMPERATURE
        if prompt: 
            kwargs["prompt"] = prompt
        
        result = client.audio.transcriptions.create(**kwargs)
        text = (result.text or "").strip()
        
        clean_text = sanitize_transcript(text, prompt)
        clean_text = _post_process_transcript(clean_text)
        
        if Config.EN_ONLY_STRICT and clean_text and not _english_like(clean_text):
            return ""
        
        return clean_text
        
    except Exception as e:
        last_error_ref[0] = f"OpenAI: {e}"
        log.error("Whisper error:\n%s", traceback.format_exc())
        return ""
    finally:
        try: 
            tmp.close()
        except: 
            pass
        try: 
            os.remove(tmp.name)
        except: 
            pass

def should_drop_low_confidence(text: str, seg_rms: float, seg_ms: int, gate_rms: float) -> bool:
    """Check if transcript should be dropped as low confidence hallucination"""
    if not Config.ASR_DROP_LOWCONF_SINGLE_ACTION:
        return False
    
    _mono = {"release","stop","next","cancel","pick"}
    tnorm = text.strip().lower()
    
    if (tnorm in _mono or re.match(r'^(release|pick|stop|next|cancel)\s+(cup|bottle|mug|can)$', tnorm)):
        if seg_rms < gate_rms * Config.ASR_LOWCONF_RMS_MULT and seg_ms <= Config.ASR_LOWCONF_MAX_MS:
            return True
    
    return False

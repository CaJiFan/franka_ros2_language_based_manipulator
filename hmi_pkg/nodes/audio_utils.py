#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Audio utilities module - handles audio processing functions
"""

import math
import wave
from array import array
from typing import Tuple

def rms_int16(pcm: bytes) -> float:
    """Calculate RMS (Root Mean Square) of PCM16 audio"""
    arr = array('h')
    arr.frombytes(pcm[:len(pcm)//2*2])
    if not arr:
        return 0.0
    mean = sum(v*v for v in arr) / len(arr)
    return math.sqrt(mean) / 32768.0

def ms_of_bytes(pcm_bytes: int, sr: int = 16000) -> int:
    """Convert byte count to milliseconds"""
    frames = pcm_bytes // 2
    return int((frames / max(1, sr)) * 1000)

def amplify_pcm16(pcm: bytes, gain: float = 1.0) -> bytes:
    """Amplify PCM16 audio by gain factor"""
    if gain <= 1.0:
        return pcm
    
    arr = array('h')
    arr.frombytes(pcm[:len(pcm)//2*2])
    for i in range(len(arr)):
        v = int(arr[i] * gain)
        if v > 32767:
            v = 32767
        if v < -32768:
            v = -32768
        arr[i] = v
    return arr.tobytes()

def write_wav(path: str, pcm: bytes, sr: int):
    """Write PCM16 audio to WAV file"""
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(sr)
        wf.writeframes(pcm)

def calculate_gain(rms: float, target_rms: float = 0.03, max_gain: float = 3.0) -> float:
    """Calculate appropriate gain for audio amplification"""
    if rms >= target_rms:
        return 1.0
    return min(max_gain, target_rms / max(0.001, rms))

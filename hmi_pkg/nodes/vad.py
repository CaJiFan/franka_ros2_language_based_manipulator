#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Voice Activity Detection (VAD) module
"""

import logging
from dataclasses import dataclass, field
from typing import Optional
from config import Config
from audio_utils import rms_int16, ms_of_bytes

log = logging.getLogger("hmi.vad")

@dataclass
class VADState:
    """VAD state management"""
    calibrating: bool = True
    calib_ms: int = 0
    calib_acc_rms: float = 0.0
    noise_rms: float = 0.0
    gate_rms: float = Config.WHISPER_MIN_RMS
    in_voice: bool = False
    voice_ms: int = 0
    sil_ms: int = 0
    speech_buf: bytearray = field(default_factory=bytearray)

class VADProcessor:
    """Voice Activity Detection processor"""
    
    def __init__(self):
        self.state = VADState()
        
    def update_gate(self):
        """Update VAD gate threshold based on noise floor"""
        self.state.gate_rms = max(
            Config.WHISPER_MIN_RMS,
            self.state.noise_rms * Config.VAD_GATE_MULT,
            Config.VAD_MIN_RMS_FLOOR
        )
        
    def start_voice(self):
        """Start voice activity"""
        self.state.in_voice = True
        self.state.sil_ms = 0
        self.state.speech_buf.clear()
        
    def end_voice(self, sr: int = Config.SR_DEFAULT) -> Optional[bytes]:
        """End voice activity and return speech segment"""
        self.state.in_voice = False
        self.state.voice_ms = 0
        self.state.sil_ms = 0
        segment = bytes(self.state.speech_buf)
        self.state.speech_buf.clear()
        
        seg_ms = ms_of_bytes(len(segment), sr)
        if seg_ms < Config.VAD_MIN_MS:
            return None
        return segment
    
    def process_calibration(self, pcm: bytes, sr: int = Config.SR_DEFAULT) -> bool:
        """
        Process audio chunk for calibration
        Returns True when calibration is complete
        """
        chunk_ms = ms_of_bytes(len(pcm), sr)
        chunk_rms = rms_int16(pcm)
        
        self.state.calib_ms += chunk_ms
        self.state.calib_acc_rms += chunk_rms
        
        if self.state.calib_ms >= int(Config.VAD_CALIB_SEC * 1000):
            samples = self.state.calib_ms / (chunk_ms or 1)
            self.state.noise_rms = self.state.calib_acc_rms / max(1, samples)
            self.update_gate()
            self.state.calibrating = False
            log.info(f"Calibration done: noise_rms={self.state.noise_rms:.4f}, gate_rms={self.state.gate_rms:.4f}")
            return True
        return False
    
    def process_audio(self, pcm: bytes, sr: int = Config.SR_DEFAULT) -> Optional[bytes]:
        """
        Process audio chunk through VAD
        Returns speech segment when complete, None otherwise
        """
        chunk_ms = ms_of_bytes(len(pcm), sr)
        chunk_rms = rms_int16(pcm)
        
        # Check if this is voice
        is_voice = (chunk_rms >= self.state.gate_rms)
        
        if not self.state.in_voice:
            if is_voice:
                self.start_voice()
                self.state.speech_buf.extend(pcm)
                self.state.voice_ms += chunk_ms
        else:
            self.state.speech_buf.extend(pcm)
            self.state.voice_ms += chunk_ms
            
            # Check for end conditions
            end_for_sil = (not is_voice) and (self.state.sil_ms + chunk_ms >= Config.VAD_SIL_HANG_MS)
            end_for_len = self.state.voice_ms >= Config.VAD_MAX_UTT_MS
            
            if is_voice:
                self.state.sil_ms = 0
            else:
                self.state.sil_ms += chunk_ms
            
            if end_for_sil or end_for_len:
                return self.end_voice(sr)
        
        return None
    
    def reset_calibration(self):
        """Reset calibration state"""
        self.state = VADState()
        log.info("VAD calibration reset")

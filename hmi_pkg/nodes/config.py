#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configuration module - loads all settings from environment variables
"""

import os
from typing import Optional

class Config:
    """Centralized configuration for HMI server"""
    
    # ASR Configuration
    ASR_ENGINE = os.getenv("ASR_ENGINE", "openai_whisper").lower()
    WHISPER_MODEL = os.getenv("WHISPER_MODEL", "whisper-1")
    WHISPER_LANG = os.getenv("WHISPER_LANG", "en").strip() or "en"
    WHISPER_TEMPERATURE = float(os.getenv("WHISPER_TEMPERATURE", "0"))
    WHISPER_USE_PROMPT = os.getenv("WHISPER_USE_PROMPT", "0").lower() in ("1", "true", "yes")
    WHISPER_CHUNK_SEC = max(0.6, float(os.getenv("WHISPER_CHUNK_SEC", "1.6")))
    WHISPER_MIN_RMS = float(os.getenv("WHISPER_MIN_RMS", "0.01"))
    WHISPER_BYPASS_RMS = os.getenv("WHISPER_BYPASS_RMS", "0").lower() in ("1", "true", "yes")
    
    # System Configuration
    CAPABILITIES_PATH = os.getenv("CAPABILITIES_PATH", "/app/hmi_pkg/config/capabilities.json")
    SR_DEFAULT = 16000
    OPENAI_KEY = os.getenv("OPENAI_API_KEY", "").strip()
    
    # Deduplication and Filtering
    ASR_MIN_MS = int(os.getenv("ASR_MIN_MS", "300"))
    ASR_DEDUP_MS = int(os.getenv("ASR_DEDUP_MS", "1700"))
    EN_ONLY_STRICT = os.getenv("EN_ONLY_STRICT", "1").lower() in ("1", "true", "yes")
    
    # VAD Configuration
    VAD_CALIB_SEC = float(os.getenv("VAD_CALIB_SEC", "0.7"))
    VAD_GATE_MULT = float(os.getenv("VAD_GATE_MULT", "2.5"))
    VAD_MIN_RMS_FLOOR = float(os.getenv("VAD_MIN_RMS_FLOOR", "0.0015"))
    VAD_MIN_MS = int(os.getenv("VAD_MIN_MS", "180"))
    VAD_SIL_HANG_MS = int(os.getenv("VAD_SIL_HANG_MS", "450"))
    VAD_MAX_UTT_MS = int(os.getenv("VAD_MAX_UTT_MS", "6000"))
    
    # Intent Configuration
    INTENT_MERGE_MS = int(os.getenv("INTENT_MERGE_MS", "900"))
    LOCK_ON_INTENT = os.getenv("LOCK_ON_INTENT", "1").lower() in ("1", "true", "yes")
    
    # ROS Configuration
    ROS_ENABLED = os.getenv("HMI_ROS_ENABLE", "1").strip().lower() in ("1", "true", "yes")
    ROS_ASR_TOPIC = os.getenv("HMI_ROS_ASR_TOPIC", "/hmi/asr_text")
    ROS_INTENT_TOPIC = os.getenv("HMI_ROS_INTENT_TOPIC", "/hmi/intent_raw")
    ROS_CONFIRM_MODE = os.getenv("HMI_ROS_CONFIRM_MODE", "intent").lower()
    
    # HTML/Static paths
    HERE = os.path.dirname(os.path.abspath(__file__))
    DEFAULT_PUBLIC_DIR = os.path.join(HERE, "..", "public")
    INDEX_HTML_PATH = os.getenv("HMI_INDEX_HTML", os.path.join(DEFAULT_PUBLIC_DIR, "index.html"))
    
    # NLU Phonetic Thresholds
    NLU_ACTION_THRESHOLD = float(os.getenv("NLU_ACTION_THRESHOLD", "0.75"))
    NLU_OBJECT_THRESHOLD = float(os.getenv("NLU_OBJECT_THRESHOLD", "0.75"))
    
    # Low confidence detection
    ASR_DROP_LOWCONF_SINGLE_ACTION = os.getenv("ASR_DROP_LOWCONF_SINGLE_ACTION", "1").lower() in ("1", "true", "yes")
    ASR_LOWCONF_RMS_MULT = float(os.getenv("ASR_LOWCONF_RMS_MULT", "1.25"))
    ASR_LOWCONF_MAX_MS = int(os.getenv("ASR_LOWCONF_MAX_MS", "1000"))

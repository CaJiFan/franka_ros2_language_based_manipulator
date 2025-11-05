#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
WebSocket handler module - manages WebSocket connections and audio processing
FIXED: Single-word actions now treated as complete intents
"""

import json
import time
import logging
from typing import Dict, Optional
from fastapi import WebSocket
from config import Config
from state import STATE, broadcast, sig_utter, norm_txt, is_confirmable, reset_pending_intent
from vad import VADProcessor
from asr_whisper import transcribe_with_openai, should_drop_low_confidence
from audio_utils import rms_int16, ms_of_bytes
from ros_bridge import get_ros_thread

log = logging.getLogger("hmi.websocket")

# Actions that don't require objects
STANDALONE_ACTIONS = {"next", "stop", "cancel", "release"}

class WebSocketHandler:
    """Handles WebSocket connections for audio streaming"""
    
    def __init__(self, nlu_model):
        self.nlu_model = nlu_model
        self.vad_processor = VADProcessor()
        self.audio_chunk_count = 0
        self.total_bytes_received = 0
        
    async def handle_connection(self, ws: WebSocket):
        """Main WebSocket connection handler"""
        await ws.accept()
        STATE.clients.add(ws)
        log.info("WebSocket client connected. Total clients: %d", len(STATE.clients))
        
        # Reset counters
        self.audio_chunk_count = 0
        self.total_bytes_received = 0
        
        # Send initial status to client
        await ws.send_text(json.dumps({
            "type": "status",
            "calibrating": self.vad_processor.state.calibrating,
            "locked": STATE.locked
        }))
        
        try:
            while True:
                msg = await ws.receive()
                if "bytes" in msg:
                    await self.handle_audio(ws, msg["bytes"])
                elif "text" in msg:
                    await self.handle_text(ws, msg["text"])
        except Exception as e:
            log.error(f"WebSocket error: {e}")
        finally:
            STATE.clients.discard(ws)
            log.info(f"WebSocket client disconnected. Total clients: {len(STATE.clients)}, Chunks received: {self.audio_chunk_count}, Total bytes: {self.total_bytes_received}")
    
    async def handle_text(self, ws: WebSocket, text: str):
        """Handle text messages from client"""
        try:
            data = json.loads(text)
            msg_type = data.get("type")
            log.info(f"Received text message: type={msg_type}")
            
            if msg_type == "meta":
                STATE.sr = int(data.get("sr", Config.SR_DEFAULT))
                log.info(f"Sample rate set to: {STATE.sr}")
        except Exception as e:
            log.error(f"Error handling text message: {e}")
    
    async def handle_audio(self, ws: WebSocket, pcm: bytes):
        """Handle audio data from client"""
        # Count chunks and bytes
        self.audio_chunk_count += 1
        self.total_bytes_received += len(pcm)
        
        STATE.bytes_total += len(pcm)
        STATE.last_rms = rms_int16(pcm)
        
        # Send RMS and bytes info to client periodically
        if self.audio_chunk_count % 10 == 0:
            await ws.send_text(json.dumps({
                "type": "rms",
                "value": STATE.last_rms
            }))
            await ws.send_text(json.dumps({
                "type": "bytes",
                "total": STATE.bytes_total
            }))
        
        # Log first few chunks with details
        if self.audio_chunk_count <= 5 or self.audio_chunk_count % 100 == 0:
            chunk_ms = ms_of_bytes(len(pcm), STATE.sr)
            log.info(f"Audio chunk #{self.audio_chunk_count}: {len(pcm)} bytes, {chunk_ms}ms, RMS={STATE.last_rms:.4f}")
        
        # Skip if locked
        if STATE.locked:
            if self.audio_chunk_count % 50 == 0:
                log.debug("System locked, skipping audio")
            return
        
        # Handle calibration phase
        if self.vad_processor.state.calibrating:
            if self.audio_chunk_count % 10 == 0:
                log.info(f"Calibrating... {self.vad_processor.state.calib_ms}ms / {int(Config.VAD_CALIB_SEC * 1000)}ms")
            
            if self.vad_processor.process_calibration(pcm, STATE.sr):
                log.info(f"✓ VAD calibration complete after {self.audio_chunk_count} chunks!")
                log.info(f"  Noise RMS: {self.vad_processor.state.noise_rms:.4f}")
                log.info(f"  Gate RMS: {self.vad_processor.state.gate_rms:.4f}")
                
                # Notify client that calibration is done
                await ws.send_text(json.dumps({
                    "type": "calibrated",
                    "noise_rms": self.vad_processor.state.noise_rms,
                    "gate_rms": self.vad_processor.state.gate_rms
                }))
            return
        
        # Log VAD state occasionally
        if self.audio_chunk_count % 50 == 0:
            log.info(f"VAD state: in_voice={self.vad_processor.state.in_voice}, voice_ms={self.vad_processor.state.voice_ms}, sil_ms={self.vad_processor.state.sil_ms}")
        
        # Process through VAD
        segment = self.vad_processor.process_audio(pcm, STATE.sr)
        if not segment:
            # Log when voice starts
            if self.vad_processor.state.in_voice and self.vad_processor.state.voice_ms < 100:
                log.info(f"Voice activity started! RMS={STATE.last_rms:.4f} >= gate={self.vad_processor.state.gate_rms:.4f}")
            return
        
        # Log segment detection
        seg_ms = ms_of_bytes(len(segment), STATE.sr)
        seg_rms = rms_int16(segment)
        log.info(f"🎤 Speech segment detected: {seg_ms}ms, RMS={seg_rms:.4f}, {len(segment)} bytes")
        
        # Transcribe the segment
        log.info("Sending to OpenAI Whisper for transcription...")
        last_error_ref = [STATE.last_error]
        text = await transcribe_with_openai(segment, STATE.sr, last_error_ref)
        STATE.last_error = last_error_ref[0]
        
        now = time.time()
        
        # Log transcription result
        if text:
            log.info(f"✓ Transcribed: '{text}'")
        else:
            log.warning(f"✗ Empty transcription. Error: {STATE.last_error if STATE.last_error else 'None'}")
        
        # Handle empty transcript
        if not text:
            await self.handle_empty_transcript(ws, now)
            return
        
        # Check for low confidence hallucination
        if should_drop_low_confidence(text, seg_rms, seg_ms, self.vad_processor.state.gate_rms):
            log.info(f"⚠ Dropped low-confidence transcript: '{text}' (RMS={seg_rms:.4f}, duration={seg_ms}ms)")
            return
        
        # Check for duplicate
        norm = norm_txt(text)
        if STATE.last_tx_norm and norm == STATE.last_tx_norm:
            time_since_last = (now - STATE.last_tx_time) * 1000
            if time_since_last < Config.ASR_DEDUP_MS:
                log.info(f"⚠ Duplicate transcript dropped: '{text}' (within {time_since_last:.0f}ms)")
                return
        
        STATE.last_tx_norm = norm
        STATE.last_tx_time = now
        STATE.last_text = text
        
        # Generate utterance signature
        STATE.utter_seq += 1
        STATE.current_sig = sig_utter(STATE.utter_seq)
        
        log.info(f"→ Sending transcript to client: '{text}'")
        
        # Send transcript to client
        await ws.send_text(json.dumps({
            "type": "final",
            "text": text,
            "sig": STATE.current_sig
        }))
        
        # Parse intent with NLU
        parsed = self.nlu_model.parse(text)
        if parsed:
            action = parsed.get("action")
            obj = parsed.get("object")
            
            # Only accept if we have a clear match (not just phonetic guessing)
            # Reject very short transcripts that don't match known commands closely
            if len(text.strip()) <= 3 and (action or obj):
                log.info(f"⚠ Rejected likely false match: '{text}' -> {parsed}")
                return
            
            log.info(f"✓ Parsed intent: action={action}, object={obj}")
            await self.handle_parsed_intent(ws, parsed, now)
        else:
            log.info(f"⚠ No intent parsed from: '{text}'")
    
    async def handle_empty_transcript(self, ws: WebSocket, now: float):
        """Handle case when transcript is empty"""
        # Check for pending intent timeout
        if STATE.pending_action or STATE.pending_object:
            time_pending = (now - STATE.pending_ts) * 1000
            if time_pending >= Config.INTENT_MERGE_MS:
                log.info(f"⏱ Pending intent timeout ({time_pending:.0f}ms), sending partial intent")
                pending_intent = {
                    "action": STATE.pending_action,
                    "object": STATE.pending_object
                }
                reset_pending_intent()
                
                STATE.utter_seq += 1
                STATE.current_sig = sig_utter(STATE.utter_seq)
                
                await ws.send_text(json.dumps({
                    "type": "intent",
                    "intent": pending_intent,
                    "sig": STATE.current_sig
                }))
                
                if Config.LOCK_ON_INTENT and is_confirmable(pending_intent):
                    STATE.locked = True
                    await broadcast({"type": "locked", "value": True})
                    log.info("🔒 System locked after intent")
        elif STATE.last_error:
            log.info(f"→ Sending error to client: {STATE.last_error}")
            await ws.send_text(json.dumps({
                "type": "error", 
                "text": STATE.last_error
            }))
            STATE.last_error = ""
    
    def is_complete_intent(self, action: Optional[str], object_val: Optional[str]) -> bool:
        """
        Check if an intent is complete.
        Standalone actions don't need objects.
        Special case: "cancel" with "can" object should be treated as standalone "cancel"
        """
        # Special case: cancel + can -> just cancel
        if action == "cancel" and object_val == "can":
            return True  # Will be handled as cancel only
        
        if action and object_val:
            # Both action and object present
            return True
        
        if action and not object_val:
            # Action without object - check if it's standalone
            return action.lower() in STANDALONE_ACTIONS
        
        return False
    
    async def handle_parsed_intent(self, ws: WebSocket, parsed: Dict, now: float):
        """Handle parsed intent from NLU"""
        action = parsed.get("action")
        object_val = parsed.get("object")
        
        # Special case: "cancel" often gets "can" as object due to phonetics
        # Treat "cancel" + "can" as just "cancel"
        if action == "cancel" and object_val == "can":
            log.info(f"ℹ️  Interpreting 'cancel can' as standalone 'cancel' command")
            object_val = None
            parsed = {"action": action, "object": None}
        
        # Check if this is a complete intent
        if self.is_complete_intent(action, object_val):
            log.info(f"✓ Complete intent: action={action}, object={object_val}")
            reset_pending_intent()
            STATE.last_intent = parsed
            
            await ws.send_text(json.dumps({
                "type": "intent",
                "intent": parsed,
                "sig": STATE.current_sig
            }))
            
            if Config.LOCK_ON_INTENT and is_confirmable(parsed):
                STATE.locked = True
                await broadcast({"type": "locked", "value": True})
                log.info("🔒 System locked after complete intent")
            
            # Publish to ROS if enabled
            ros_thread = get_ros_thread()
            if ros_thread and ros_thread.ready:
                intent_json = json.dumps({
                    "action": action,
                    "object": {"label": object_val} if object_val else None,
                    "allow_execution": False
                }, separators=(',', ':'))
                ros_thread.publish_intent(intent_json)
                log.info(f"→ Published to ROS: {intent_json}")
        
        # Partial intent - try to merge
        else:
            log.info(f"⚠ Partial intent: action={action}, object={object_val}")
            await self.handle_partial_intent(ws, action, object_val, now)
    
    async def handle_partial_intent(self, ws: WebSocket, action: Optional[str], 
                                   object_val: Optional[str], now: float):
        """Handle partial intent and attempt merging"""
        merged = None
        
        if action and not object_val:
            # Action without object (for actions that require objects like "pick")
            if STATE.pending_object and (now - STATE.pending_ts) * 1000 <= Config.INTENT_MERGE_MS:
                log.info(f"✓ Merged action '{action}' with pending object '{STATE.pending_object}'")
                merged = {"action": action, "object": STATE.pending_object}
                reset_pending_intent()
            else:
                log.info(f"⏳ Storing pending action: {action}")
                STATE.pending_action = action
                STATE.pending_object = None
                STATE.pending_ts = now
        
        elif object_val and not action:
            # Object without action
            if STATE.pending_action and (now - STATE.pending_ts) * 1000 <= Config.INTENT_MERGE_MS:
                log.info(f"✓ Merged object '{object_val}' with pending action '{STATE.pending_action}'")
                merged = {"action": STATE.pending_action, "object": object_val}
                reset_pending_intent()
            else:
                log.info(f"⏳ Storing pending object: {object_val}")
                STATE.pending_object = object_val
                STATE.pending_action = None
                STATE.pending_ts = now
        
        # Send merged intent if we have one
        if merged:
            STATE.last_intent = merged
            await ws.send_text(json.dumps({
                "type": "intent",
                "intent": merged,
                "sig": STATE.current_sig
            }))
            
            if Config.LOCK_ON_INTENT and is_confirmable(merged):
                STATE.locked = True
                await broadcast({"type": "locked", "value": True})
                log.info("🔒 System locked after merged intent")

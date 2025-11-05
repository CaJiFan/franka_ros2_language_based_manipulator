#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
HMI Voice Control Server - Main Application (Refactored)
FIXED: Confirm endpoint now properly unlocks and broadcasts
"""

import os
import json
import logging
from fastapi import FastAPI, WebSocket, Form, HTTPException
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

# Import all our refactored modules
from config import Config
from state import STATE, broadcast, is_confirmable
from nlu_engine import load_capabilities
from websocket_handler import WebSocketHandler
from ros_bridge import initialize_ros, get_ros_thread
from vad import VADProcessor

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("hmi")

# Initialize NLU model
NLU_MODEL = load_capabilities()

# Initialize WebSocket handler
ws_handler = WebSocketHandler(NLU_MODEL)

# Create FastAPI app
app = FastAPI()
app.add_middleware(
    CORSMiddleware, 
    allow_origins=["*"], 
    allow_credentials=True, 
    allow_methods=["*"], 
    allow_headers=["*"]
)

# Mount static files if directory exists
if os.path.isdir(Config.DEFAULT_PUBLIC_DIR):
    app.mount("/static", StaticFiles(directory=Config.DEFAULT_PUBLIC_DIR), name="static")

@app.on_event("startup")
async def on_startup():
    """Initialize system on startup"""
    # Reset VAD calibration
    ws_handler.vad_processor.reset_calibration()
    
    # Initialize ROS if enabled
    initialize_ros()

@app.get("/", response_class=FileResponse)
async def index():
    """Serve main HTML interface"""
    path = Config.INDEX_HTML_PATH
    if not os.path.exists(path):
        raise HTTPException(status_code=404, detail=f"Index HTML not found at {path}. Set HMI_INDEX_HTML or place public/index.html.")
    return FileResponse(path, media_type="text/html")

@app.post("/unlock")
async def unlock():
    """Unlock the system"""
    log.info("🔓 Unlocking system")
    STATE.locked = False
    await broadcast({"type": "locked", "value": False})
    return {"locked": False}

@app.post("/discard")
async def discard_intent():
    """Discard the current intent and unlock"""
    log.info("🗑️  Discarding current intent")
    STATE.locked = False
    STATE.last_intent = None
    await broadcast({"type": "locked", "value": False})
    await broadcast({"type": "discarded"})
    return {"ok": True, "discarded": True, "locked": False}

@app.post("/confirm")
async def confirm_intent(sig: str = Form(None)):
    """Confirm an intent for execution"""
    log.info(f"Confirm request received: sig={sig}, current_sig={STATE.current_sig}")
    
    # If no signature provided, use the current one
    if not sig:
        sig = STATE.current_sig
        log.info(f"No signature provided, using current: {sig}")
    
    # Check if already confirmed
    if sig and sig == STATE.last_confirmed_sig:
        log.info(f"Command already confirmed: {sig}")
        return {"ok": True, "confirmed": sig, "duplicate": True, "locked": False}
    
    # Only check signature mismatch if one was provided
    if sig and STATE.current_sig and sig != STATE.current_sig:
        log.warning(f"Signature mismatch: got {sig}, expected {STATE.current_sig}")
        return {"ok": False, "error": "Signature mismatch"}
    
    STATE.last_confirmed_sig = sig if sig else STATE.current_sig
    
    log.info(f"✓ Confirming command")
    STATE.locked = False
    
    # Broadcast unlock to all clients
    await broadcast({"type": "locked", "value": False})
    
    # Publish to ROS if enabled
    ros_thread = get_ros_thread()
    if ros_thread and ros_thread.ready:
        if Config.ROS_CONFIRM_MODE == "intent":
            if STATE.last_intent:
                intent_out = json.dumps({
                    "action": STATE.last_intent.get("action"),
                    "object": {"label": STATE.last_intent.get("object")} if STATE.last_intent.get("object") else None,
                    "allow_execution": True
                }, separators=(',', ':'))
                ros_thread.publish_intent(intent_out)
                log.info(f"→ Published confirmed intent to ROS: {intent_out}")
        else:
            if STATE.last_text:
                ros_thread.publish_asr(STATE.last_text)
                log.info(f"→ Published confirmed text to ROS: {STATE.last_text}")
    
    # Broadcast confirmation to all clients
    await broadcast({"type": "confirmed", "sig": sig if sig else STATE.current_sig})
    
    log.info(f"✓ Confirmation complete, system unlocked")
    return {"ok": True, "confirmed": sig if sig else STATE.current_sig, "locked": False}

@app.get("/status")
async def status():
    """Health check and status endpoint"""
    ros_thread = get_ros_thread()
    return {
        "status": "healthy",
        "asr_engine": Config.ASR_ENGINE,
        "ros_enabled": Config.ROS_ENABLED,
        "ros_ready": ros_thread.ready if ros_thread else False,
        "openai_configured": bool(Config.OPENAI_KEY),
        "connected_clients": len(STATE.clients),
        "locked": STATE.locked,
        "last_text": STATE.last_text,
        "last_intent": STATE.last_intent
    }

@app.get("/debug")
async def debug_info():
    """Debug endpoint with current state information"""
    ros_thread = get_ros_thread()
    return {
        "state": {
            "locked": STATE.locked,
            "calibrating": ws_handler.vad_processor.state.calibrating,
            "noise_rms": ws_handler.vad_processor.state.noise_rms,
            "gate_rms": ws_handler.vad_processor.state.gate_rms,
            "last_text": STATE.last_text,
            "last_intent": STATE.last_intent,
            "connected_clients": len(STATE.clients),
            "pending_action": STATE.pending_action,
            "pending_object": STATE.pending_object
        },
        "nlu": {
            "actions": list(NLU_MODEL.actions),
            "objects": list(NLU_MODEL.objects)
        },
        "config": {
            "asr_engine": Config.ASR_ENGINE,
            "model": Config.WHISPER_MODEL,
            "chunk_sec": Config.WHISPER_CHUNK_SEC,
            "vad_enabled": True,
            "ros_enabled": Config.ROS_ENABLED,
            "ros_ready": ros_thread.ready if ros_thread else False,
            "lock_on_intent": Config.LOCK_ON_INTENT
        }
    }

@app.get("/debug/audio")
async def debug_audio():
    """Audio debug interface"""
    html = """<!DOCTYPE html>
<html>
<head>
    <title>HMI Audio Debug</title>
    <style>
        body { 
            font-family: Arial, sans-serif; 
            max-width: 900px; 
            margin: 0 auto; 
            padding: 20px;
            background: #f5f5f5;
        }
        .container {
            background: white;
            padding: 20px;
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        button { 
            padding: 12px 24px; 
            margin: 5px; 
            font-size: 16px;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            transition: all 0.3s;
        }
        button:hover { opacity: 0.8; }
        #startBtn { background: #4CAF50; color: white; }
        #stopBtn { background: #f44336; color: white; }
        #unlockBtn { background: #FF9800; color: white; }
        #startBtn:disabled, #stopBtn:disabled, #unlockBtn:disabled { 
            opacity: 0.3; 
            cursor: not-allowed;
        }
        #status { 
            margin: 20px 0; 
            padding: 15px; 
            background: #e3f2fd;
            border-left: 4px solid #2196F3;
            border-radius: 4px;
            font-weight: bold;
        }
        #transcript { 
            margin-top: 20px;
            max-height: 400px;
            overflow-y: auto;
        }
        .transcript-item { 
            margin: 10px 0; 
            padding: 12px; 
            background: #fff; 
            border-left: 4px solid #2196F3;
            border-radius: 4px;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }
        .intent { 
            color: #4CAF50; 
            font-weight: bold;
            border-left-color: #4CAF50;
        }
        .error {
            color: #f44336;
            border-left-color: #f44336;
        }
        .locked {
            background: #fff3e0;
            border-left-color: #FF9800;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🎤 HMI Voice Control - Audio Debug</h1>
        <div>
            <button id="startBtn">Start Recording</button>
            <button id="stopBtn" disabled>Stop</button>
            <button id="unlockBtn" disabled>🔓 Unlock System</button>
        </div>
        <div id="status">Ready to start</div>
        <div id="transcript"></div>
    </div>
    
    <script>
    let ws, audioContext, source, processor;
    let isLocked = false;
    const startBtn = document.getElementById('startBtn');
    const stopBtn = document.getElementById('stopBtn');
    const unlockBtn = document.getElementById('unlockBtn');
    const status = document.getElementById('status');
    const transcript = document.getElementById('transcript');
    
    function updateStatus(msg, isError = false) {
        status.textContent = msg;
        status.style.background = isError ? '#ffebee' : '#e3f2fd';
        status.style.borderLeftColor = isError ? '#f44336' : '#2196F3';
    }
    
    function addTranscript(msg, type = 'normal') {
        const div = document.createElement('div');
        div.className = 'transcript-item ' + type;
        div.innerHTML = '<strong>' + new Date().toLocaleTimeString() + '</strong> - ' + msg;
        transcript.insertBefore(div, transcript.firstChild);
    }
    
    startBtn.onclick = async () => {
        try {
            const stream = await navigator.mediaDevices.getUserMedia({audio: true});
            ws = new WebSocket('ws://' + window.location.host + '/asr');
            
            ws.onopen = () => {
                updateStatus('✓ Connected - Calibrating... (speak after 1 second)');
                ws.send(JSON.stringify({type:'meta',sr:16000}));
                addTranscript('WebSocket connected', 'normal');
            };
            
            ws.onmessage = (e) => {
                const msg = JSON.parse(e.data);
                console.log('Received:', msg);
                
                if(msg.type === 'calibrated') {
                    updateStatus('✓ Calibrated - Ready to listen!');
                    addTranscript('Calibration complete. Speak now!', 'normal');
                }
                else if(msg.type === 'final') {
                    updateStatus('Transcribed: ' + msg.text);
                    addTranscript('Transcript: <strong>' + msg.text + '</strong>', 'normal');
                } 
                else if(msg.type === 'intent') {
                    const intent = msg.intent;
                    const intentStr = intent.action + (intent.object ? ' → ' + intent.object : '');
                    updateStatus('Intent detected: ' + intentStr);
                    addTranscript('Intent: <strong>' + intentStr + '</strong>', 'intent');
                }
                else if(msg.type === 'locked') {
                    isLocked = msg.value;
                    unlockBtn.disabled = !isLocked;
                    if(isLocked) {
                        updateStatus('🔒 System locked - Click Unlock to continue');
                        addTranscript('System locked - awaiting confirmation', 'locked');
                    } else {
                        updateStatus('🔓 System unlocked - Ready for commands');
                        addTranscript('System unlocked', 'normal');
                    }
                }
                else if(msg.type === 'confirmed') {
                    updateStatus('✓ Command confirmed and executed');
                    addTranscript('Command confirmed', 'intent');
                }
                else if(msg.type === 'error') {
                    updateStatus('Error: ' + msg.text, true);
                    addTranscript('Error: ' + msg.text, 'error');
                }
            };
            
            ws.onerror = (err) => {
                console.error('WebSocket error:', err);
                updateStatus('WebSocket error occurred', true);
            };
            
            ws.onclose = () => {
                updateStatus('Disconnected');
                addTranscript('WebSocket disconnected', 'error');
            };
            
            audioContext = new AudioContext({sampleRate: 16000});
            source = audioContext.createMediaStreamSource(stream);
            processor = audioContext.createScriptProcessor(4096, 1, 1);
            
            processor.onaudioprocess = (e) => {
                if (ws.readyState === WebSocket.OPEN) {
                    const float32 = e.inputBuffer.getChannelData(0);
                    const int16 = new Int16Array(float32.length);
                    for(let i = 0; i < float32.length; i++) {
                        int16[i] = Math.max(-32768, Math.min(32767, Math.floor(float32[i] * 32768)));
                    }
                    ws.send(int16.buffer);
                }
            };
            
            source.connect(processor);
            processor.connect(audioContext.destination);
            
            startBtn.disabled = true;
            stopBtn.disabled = false;
        } catch(err) {
            console.error('Error starting:', err);
            updateStatus('Error: ' + err.message, true);
        }
    };
    
    stopBtn.onclick = () => {
        if(ws) ws.close();
        if(source) source.disconnect();
        if(processor) processor.disconnect();
        if(audioContext) audioContext.close();
        startBtn.disabled = false;
        stopBtn.disabled = true;
        unlockBtn.disabled = true;
        isLocked = false;
        updateStatus('Stopped');
    };
    
    unlockBtn.onclick = async () => {
        try {
            updateStatus('Unlocking system...');
            const response = await fetch('/unlock', { method: 'POST' });
            const data = await response.json();
            if(!data.locked) {
                updateStatus('✓ System unlocked');
                addTranscript('Manual unlock successful', 'normal');
                isLocked = false;
                unlockBtn.disabled = true;
            }
        } catch(err) {
            console.error('Unlock error:', err);
            updateStatus('Unlock failed: ' + err.message, true);
        }
    };
    
    // Add keyboard shortcuts
    document.addEventListener('keydown', (e) => {
        if(e.key === 'u' && isLocked) {
            unlockBtn.click();
        }
    });
    </script>
</body>
</html>"""
    return HTMLResponse(content=html)

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """Main WebSocket endpoint for audio streaming"""
    await ws_handler.handle_connection(ws)

@app.websocket("/asr")
async def websocket_asr_endpoint(ws: WebSocket):
    """Alternative WebSocket endpoint for backward compatibility"""
    await ws_handler.handle_connection(ws)

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

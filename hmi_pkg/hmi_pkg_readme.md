# HMI Voice Control Package

Voice-controlled Human-Machine Interface for Franka Emika robot using ROS2, Natural Language Understanding, and OpenAI Whisper ASR.

## 🚀 Quick Start

### Using Docker (Recommended)

```bash
cd hmi_pkg

# Set your OpenAI API key
export OPENAI_API_KEY='sk-your-api-key-here'

# Start the service
docker-compose up -d

# Access the web interface
open http://localhost:8000
```

### Using Python Directly

```bash
cd hmi_pkg

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export OPENAI_API_KEY='sk-your-api-key-here'
export PYTHONPATH=$PWD/nodes:$PYTHONPATH

# Run the server
python nodes/hmi_server.py
```

## 📋 Prerequisites

- **Docker**: 20.10+ (for containerized deployment)
- **Python**: 3.10+ (for local development)
- **OpenAI API Key**: Required for speech recognition
- **ROS2 Humble**: Optional, for robot integration

## 🏗️ Architecture

### Module Overview

```
hmi_pkg/
├── nodes/                      # Core application modules
│   ├── hmi_server.py          # Main FastAPI server
│   ├── websocket_handler.py   # WebSocket connection handler
│   ├── vad.py                 # Voice Activity Detection
│   ├── asr_whisper.py         # OpenAI Whisper ASR integration
│   ├── nlu_engine.py          # Natural Language Understanding
│   ├── ros_bridge.py          # ROS2 integration (optional)
│   ├── state.py               # Application state management
│   ├── audio_utils.py         # Audio processing utilities
│   └── config.py              # Configuration management
├── config/
│   └── capabilities.json      # Robot capabilities definition
├── public/
│   └── index.html             # Web-based control interface
├── Dockerfile                 # Container definition
├── docker-compose.yml         # Service orchestration
└── requirements.txt           # Python dependencies
```

### Data Flow

```
Browser (Microphone)
    ↓ WebSocket
WebSocketHandler
    ↓ Audio chunks
VADProcessor (Calibration & Segmentation)
    ↓ Speech segments
OpenAI Whisper (Transcription)
    ↓ Text
NLU Engine (Intent Parsing)
    ↓ Intent {action, object}
ROS2 Bridge (Optional)
    ↓ ROS2 Topics
Robot Controller
```

## 🎮 Usage

### Web Interface

1. **Open the interface**: Navigate to `http://localhost:8000`
2. **Start recording**: Click "🎙️ Start Recording"
3. **Speak a command**: e.g., "pick cup", "release", "next"
4. **Confirm or discard**: Click "✅ Confirm" or "❌ Discard"

### Available Commands

**Actions**: `pick`, `release`, `next`, `stop`, `cancel`

**Objects**: `cup`, `bottle`, `mug`, `can`

**Examples**:
- "pick cup" → `{action: "pick", object: "cup"}`
- "release bottle" → `{action: "release", object: "bottle"}`
- "next" → `{action: "next", object: null}`
- "stop" → `{action: "stop", object: null}`
- "cancel" → `{action: "cancel", object: null}`

### API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Web control interface |
| `/status` | GET | System health check |
| `/debug` | GET | Debug information and state |
| `/debug/audio` | GET | Audio testing interface |
| `/ws` or `/asr` | WebSocket | Audio streaming endpoint |
| `/unlock` | POST | Unlock the system |
| `/confirm` | POST | Confirm pending command |
| `/discard` | POST | Discard pending command |

### ROS2 Integration

When `HMI_ROS_ENABLE=1`, the system publishes to:

**Topics**:
- `/hmi/asr_text` (std_msgs/String) - Transcribed text
- `/hmi/intent_raw` (std_msgs/String) - Parsed intent as JSON

**Intent Format**:
```json
{
  "action": "pick",
  "object": {"label": "cup"},
  "allow_execution": true
}
```

**Listen to topics**:
```bash
# In another terminal
ros2 topic echo /hmi/asr_text
ros2 topic echo /hmi/intent_raw
```

## ⚙️ Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | - | **Required**: OpenAI API key |
| `WHISPER_MODEL` | `whisper-1` | Whisper model name |
| `WHISPER_LANG` | `en` | Language code (en, es, it, etc.) |
| `WHISPER_CHUNK_SEC` | `1.6` | Audio chunk duration (seconds) |
| `WHISPER_MIN_RMS` | `0.01` | Minimum RMS threshold |
| `VAD_CALIB_SEC` | `0.7` | Calibration duration (seconds) |
| `VAD_GATE_MULT` | `2.5` | Voice detection sensitivity |
| `VAD_MIN_MS` | `180` | Minimum utterance duration (ms) |
| `VAD_SIL_HANG_MS` | `450` | Silence detection threshold (ms) |
| `ASR_DEDUP_MS` | `1700` | Deduplication window (ms) |
| `INTENT_MERGE_MS` | `900` | Intent merging window (ms) |
| `LOCK_ON_INTENT` | `1` | Lock after intent (0=disabled, 1=enabled) |
| `HMI_ROS_ENABLE` | `0` | Enable ROS2 (0=disabled, 1=enabled) |
| `HMI_ROS_ASR_TOPIC` | `/hmi/asr_text` | ASR topic name |
| `HMI_ROS_INTENT_TOPIC` | `/hmi/intent_raw` | Intent topic name |

### Capabilities Configuration

Edit `config/capabilities.json` to customize available actions and objects:

```json
{
  "actions": ["pick", "release", "next", "stop", "cancel"],
  "objects": ["cup", "bottle", "mug", "can"],
  "synonyms": {
    "actions": {
      "pick": ["grab", "take", "get"],
      "release": ["drop", "put down"],
      "cancel": ["abort", "quit"]
    },
    "objects": {
      "cup": ["glass", "container"],
      "bottle": ["flask"],
      "mug": ["tumbler"]
    }
  }
}
```

## 🐳 Docker Commands

```bash
# Build the image
docker-compose build

# Start the service
docker-compose up -d

# View logs
docker-compose logs -f

# Stop the service
docker-compose down

# Rebuild and restart
docker-compose up -d --build

# Enter container shell
docker exec -it hmi-voice-control bash

# Check service health
curl http://localhost:8000/status
```

## 🛠️ Development

### Running Locally

```bash
# Install dependencies
pip install -r requirements.txt

# Set environment
export OPENAI_API_KEY='sk-your-key'
export PYTHONPATH=$PWD/nodes:$PYTHONPATH

# Run with auto-reload
cd nodes
uvicorn hmi_server:app --reload --host 0.0.0.0 --port 8000
```

### Testing ASR

```bash
cd nodes
python debug_asr.py

# Or test with audio file
python debug_asr.py /path/to/audio.wav
```

### Module Testing

```bash
# Test VAD
python -c "from vad import VADProcessor; print('VAD OK')"

# Test NLU
python -c "from nlu_engine import load_capabilities; print('NLU OK')"

# Test Config
python -c "from config import Config; print(Config.OPENAI_KEY[:10])"
```

## 🐛 Troubleshooting

### WebSocket Connection Failed

```bash
# Check if server is running
curl http://localhost:8000/status

# Check container logs
docker-compose logs -f

# Verify port is exposed
docker ps | grep hmi-voice-control
```

### No Transcription

1. **Check audio level**: Audio meter should show activity
2. **Verify API key**: `docker exec hmi-voice-control printenv | grep OPENAI`
3. **Check VAD calibration**: Should complete in ~1 second
4. **Increase sensitivity**: Set `WHISPER_MIN_RMS=0.001`

### Import Errors in Container

```bash
# Check PYTHONPATH
docker exec hmi-voice-control printenv PYTHONPATH

# List Python files
docker exec hmi-voice-control find /app/hmi_pkg -name "*.py"

# Check __init__.py exists
docker exec hmi-voice-control ls -la /app/hmi_pkg/nodes/__init__.py
```

### ROS2 Not Publishing

```bash
# Enable ROS in docker-compose.yml
# HMI_ROS_ENABLE=1

# Check ROS is available
docker exec hmi-voice-control bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# View logs for ROS errors
docker-compose logs | grep -i ros
```

## 📊 System Features

### Voice Activity Detection (VAD)
- Automatic noise floor calibration
- Real-time speech segmentation
- Silence detection with hang time
- Configurable sensitivity

### Speech Recognition (ASR)
- OpenAI Whisper integration
- Multi-language support
- Automatic gain control
- Low-confidence filtering
- Deduplication

### Natural Language Understanding (NLU)
- Action and object extraction
- Phonetic similarity matching
- Fuzzy string matching
- Intent merging for multi-turn commands
- Synonym support

### Safety Features
- Command confirmation required
- System lock during execution
- Discard command option
- Duplicate prevention

## 📚 Module Details

### `hmi_server.py`
Main FastAPI application with endpoints and WebSocket handling.

### `websocket_handler.py`
Manages WebSocket connections, orchestrates VAD → ASR → NLU pipeline.

### `vad.py`
Voice Activity Detection with automatic calibration and speech segmentation.

### `asr_whisper.py`
OpenAI Whisper integration for speech-to-text transcription.

### `nlu_engine.py`
Natural Language Understanding with phonetic matching and intent parsing.

### `ros_bridge.py`
Optional ROS2 integration for publishing transcripts and intents.

### `state.py`
Application state management with WebSocket client tracking.

### `audio_utils.py`
Audio processing utilities (RMS calculation, resampling, WAV writing).

### `config.py`
Centralized configuration from environment variables.

## 🔗 Integration Examples

### With ROS2 Robot Controller

```python
import rclpy
from std_msgs.msg import String
import json

class RobotController:
    def __init__(self):
        self.node = rclpy.create_node('robot_controller')
        self.sub = self.node.create_subscription(
            String, 
            '/hmi/intent_raw',
            self.intent_callback,
            10
        )
    
    def intent_callback(self, msg):
        intent = json.loads(msg.data)
        action = intent.get('action')
        obj = intent.get('object', {}).get('label')
        
        if action == 'pick' and obj:
            self.pick_object(obj)
        elif action == 'release':
            self.release_object()
        # ... handle other actions
```

### With Custom Web Application

```javascript
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onmessage = (event) => {
    const msg = JSON.parse(event.data);
    
    if (msg.type === 'intent') {
        console.log('Intent:', msg.intent);
        // Send to your robot controller
        fetch('/robot/execute', {
            method: 'POST',
            body: JSON.stringify(msg.intent)
        });
    }
};

// Send audio
navigator.mediaDevices.getUserMedia({audio: true})
    .then(stream => {
        const audioContext = new AudioContext({sampleRate: 16000});
        const source = audioContext.createMediaStreamSource(stream);
        const processor = audioContext.createScriptProcessor(4096, 1, 1);
        
        processor.onaudioprocess = (e) => {
            const float32 = e.inputBuffer.getChannelData(0);
            const int16 = convertToInt16(float32);
            ws.send(int16.buffer);
        };
        
        source.connect(processor);
        processor.connect(audioContext.destination);
    });
```

## 🔒 Security Notes

- **Never commit `.env` files** with API keys
- **Use environment variables** for sensitive data
- **Restrict network access** in production
- **Enable HTTPS** for web interface in production
- **Validate all inputs** before robot execution

## 📝 License

MIT License - See LICENSE file for details

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 🙏 Acknowledgments

- OpenAI Whisper for speech recognition
- FastAPI for the web framework
- ROS2 for robot integration
- Franka Emika for the robot platform

## 📞 Support

For issues and questions:
- Check the troubleshooting section above
- Review logs: `docker-compose logs -f`
- Test endpoints: `curl http://localhost:8000/debug`

---

**Made with ❤️ for the robotics community**

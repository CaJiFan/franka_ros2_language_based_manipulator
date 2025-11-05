#!/usr/bin/env python3
"""
ASR Debugging Tool - Test transcription independently
Run this to verify OpenAI Whisper is working correctly
"""

import os
import sys
import tempfile
import wave
from array import array

# Add the nodes directory to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import Config
from audio_utils import rms_int16, ms_of_bytes, amplify_pcm16, write_wav
from asr_whisper import transcribe_with_openai

def create_test_audio(duration_sec=2.0, sr=16000):
    """
    Create a test audio buffer with a simple tone
    This simulates audio data for testing
    """
    import math
    samples = int(duration_sec * sr)
    frequency = 440  # A4 note
    amplitude = 10000  # Moderate volume
    
    arr = array('h')
    for i in range(samples):
        sample = int(amplitude * math.sin(2 * math.pi * frequency * i / sr))
        arr.append(sample)
    
    return arr.tobytes()

async def test_transcription():
    """Test the transcription pipeline"""
    print("=" * 60)
    print("ASR Debugging Tool")
    print("=" * 60)
    print()
    
    # Check configuration
    print("Configuration:")
    print(f"  ASR_ENGINE: {Config.ASR_ENGINE}")
    print(f"  WHISPER_MODEL: {Config.WHISPER_MODEL}")
    print(f"  WHISPER_LANG: {Config.WHISPER_LANG}")
    print(f"  OPENAI_KEY set: {bool(Config.OPENAI_KEY)}")
    print(f"  WHISPER_MIN_RMS: {Config.WHISPER_MIN_RMS}")
    print(f"  WHISPER_CHUNK_SEC: {Config.WHISPER_CHUNK_SEC}")
    print()
    
    if not Config.OPENAI_KEY:
        print("âŒ ERROR: OPENAI_API_KEY not set!")
        print("Please set the OPENAI_API_KEY environment variable.")
        return 1
    
    # Create test audio
    print("Creating test audio (2 seconds of tone)...")
    pcm = create_test_audio(duration_sec=2.0)
    rms = rms_int16(pcm)
    ms = ms_of_bytes(len(pcm), Config.SR_DEFAULT)
    
    print(f"  Audio length: {ms}ms")
    print(f"  Audio RMS: {rms:.4f}")
    print()
    
    # Save test audio to file
    test_wav = "/tmp/test_audio.wav"
    write_wav(test_wav, pcm, Config.SR_DEFAULT)
    print(f"Test audio saved to: {test_wav}")
    print()
    
    # Test transcription
    print("Testing transcription with OpenAI Whisper...")
    print("(This will send a 440Hz tone - expect empty or gibberish result)")
    print()
    
    last_error = [""]
    text = await transcribe_with_openai(pcm, Config.SR_DEFAULT, last_error)
    
    if last_error[0]:
        print(f"âŒ Transcription Error: {last_error[0]}")
        return 1
    
    print(f"âœ" Transcription successful!")
    print(f"  Result: '{text}'")
    print()
    
    # Test with actual audio file if available
    print("=" * 60)
    print("To test with real audio:")
    print("1. Record yourself saying a command")
    print("2. Save as 16kHz mono WAV file")
    print("3. Run: python debug_asr.py /path/to/audio.wav")
    print("=" * 60)
    
    return 0

async def test_audio_file(filepath):
    """Test transcription with an actual audio file"""
    print(f"Testing transcription with: {filepath}")
    print()
    
    if not os.path.exists(filepath):
        print(f"âŒ File not found: {filepath}")
        return 1
    
    # Read WAV file
    try:
        with wave.open(filepath, 'rb') as wf:
            sr = wf.getframerate()
            frames = wf.readframes(wf.getnframes())
            
            print(f"Audio info:")
            print(f"  Channels: {wf.getnchannels()}")
            print(f"  Sample width: {wf.getsampwidth()}")
            print(f"  Sample rate: {sr}")
            print(f"  Frames: {wf.getnframes()}")
            print()
            
            if wf.getnchannels() != 1 or wf.getsampwidth() != 2:
                print("âŒ Audio must be mono, 16-bit PCM")
                return 1
            
            pcm = frames
    except Exception as e:
        print(f"âŒ Error reading audio file: {e}")
        return 1
    
    # Calculate audio properties
    rms = rms_int16(pcm)
    ms = ms_of_bytes(len(pcm), sr)
    
    print(f"Audio properties:")
    print(f"  Duration: {ms}ms")
    print(f"  RMS: {rms:.4f}")
    print(f"  Above threshold: {rms >= Config.WHISPER_MIN_RMS}")
    print()
    
    # Transcribe
    print("Transcribing...")
    last_error = [""]
    text = await transcribe_with_openai(pcm, sr, last_error)
    
    if last_error[0]:
        print(f"âŒ Error: {last_error[0]}")
        return 1
    
    print(f"âœ" Transcription: '{text}'")
    return 0

if __name__ == "__main__":
    import asyncio
    
    if len(sys.argv) > 1:
        # Test with provided audio file
        exit_code = asyncio.run(test_audio_file(sys.argv[1]))
    else:
        # Run basic test
        exit_code = asyncio.run(test_transcription())
    
    sys.exit(exit_code)

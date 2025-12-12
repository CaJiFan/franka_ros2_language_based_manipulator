import speech_recognition as sr

def test_microphone():
    recognizer = sr.Recognizer()

    # 1. List all available microphones to help you debug
    print("Searching for microphones...")
    mic_list = sr.Microphone.list_microphone_names()
    
    bluetooth_mic_index = None
    
    for index, name in enumerate(mic_list):
        print(f"Microphone with name \"{name}\" found for `Microphone(device_index={index})`")
        # Auto-detect a device with 'blue' or 'headset' in the name (optional helper)
        if "blue" in name.lower() or "headset" in name.lower():
            bluetooth_mic_index = index

    # Ask user to select manually if auto-detection is unsure
    print("\n---")
    if bluetooth_mic_index is not None:
        print(f"Likely Bluetooth Mic found at index {bluetooth_mic_index}. Using it.")
    else:
        print("Could not auto-detect Bluetooth mic. Using default system mic.")
        bluetooth_mic_index = None # Passing None uses the default system mic

    # 2. Record Audio
    # If device_index is None, it uses the Default Ubuntu Input device
    try:
        with sr.Microphone(device_index=bluetooth_mic_index) as source:
            print("\nAdjusting for ambient noise... (Please wait 1s)")
            recognizer.adjust_for_ambient_noise(source, duration=1)
            
            print("🎤 Listening... Speak into your Bluetooth Mic now!")
            audio = recognizer.listen(source, timeout=5, phrase_time_limit=10)
            print("✅ Recording complete. Transcribing...")

            # 3. Transcribe using Google Web Speech API (Free, requires internet)
            try:
                text = recognizer.recognize_google(audio)
                print(f"\n📝 You said: \"{text}\"")
            except sr.UnknownValueError:
                print("❌ Could not understand audio (too quiet or unclear).")
            except sr.RequestError as e:
                print(f"❌ Could not request results from Google Speech Recognition service; {e}")

    except OSError as e:
        print(f"\n❌ Error accessing microphone: {e}")
        print("Tip: Check if your Bluetooth mic is connected and selected in Ubuntu Settings > Sound.")

if __name__ == "__main__":
    test_microphone()
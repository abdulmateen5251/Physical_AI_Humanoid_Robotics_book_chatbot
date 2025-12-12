# Chapter 1: Speech Recognition with Whisper

## What is Whisper?

Whisper is OpenAI's speech recognition model that:
- **Handles multiple languages**: 99 languages supported
- **Robust to noise**: Works in noisy environments
- **Free and open**: Available via OpenAI API or locally
- **Fast**: Real-time transcription possible with quantization

## Installation and Setup

**Install required packages:**

```bash
pip install openai-whisper
pip install librosa  # Audio processing
pip install pyaudio  # Microphone input
```

**Download model:**

```bash
# Small model (774M)
whisper tiny

# Medium model (1.5GB)  
whisper base

# Large model (2.9GB)
whisper small
```

## Real-time Speech Recognition

**Capture and transcribe microphone input:**

```python
import whisper
import pyaudio
import numpy as np
from threading import Thread

class WhisperRecognizer:
    def __init__(self, model_name='base'):
        self.model = whisper.load_model(model_name)
        self.CHUNK = 1024
        self.FORMAT = pyaudio.paFloat32
        self.CHANNELS = 1
        self.RATE = 16000
        self.recording = False
        self.audio_data = []
    
    def record_audio(self, duration=5):
        """Record audio for specified duration (seconds)"""
        p = pyaudio.PyAudio()
        stream = p.open(
            format=self.FORMAT,
            channels=self.CHANNELS,
            rate=self.RATE,
            input=True,
            frames_per_buffer=self.CHUNK
        )
        
        print(f"Recording for {duration} seconds...")
        
        frames = []
        for _ in range(0, int(self.RATE / self.CHUNK * duration)):
            data = stream.read(self.CHUNK)
            frames.append(np.frombuffer(data, dtype=np.float32))
        
        stream.stop_stream()
        stream.close()
        p.terminate()
        
        return np.concatenate(frames)
    
    def transcribe(self, audio_data):
        """Transcribe audio to text"""
        result = self.model.transcribe(audio_data, language="en")
        return result['text']
    
    def recognize(self, duration=5):
        """Record and transcribe in one call"""
        audio = self.record_audio(duration)
        text = self.transcribe(audio)
        return text

# Example usage
if __name__ == '__main__':
    recognizer = WhisperRecognizer()
    command = recognizer.recognize(duration=5)
    print(f"You said: {command}")
```

## Integration with ROS 2

**Create a ROS 2 node for voice commands:**

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command_node')
        self.recognizer = WhisperRecognizer(model_name='base')
        
        # Publisher for recognized commands
        self.publisher = self.create_publisher(
            String,
            '/voice_commands',
            10
        )
        
        # Timer to continuously listen
        self.timer = self.create_timer(
            10.0,  # Listen every 10 seconds
            self.listen_callback
        )
    
    def listen_callback(self):
        try:
            command = self.recognizer.recognize(duration=5)
            self.get_logger().info(f"Command: {command}")
            
            msg = String()
            msg.data = command
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Recognition error: {e}")

def main():
    rclpy.init()
    node = VoiceCommandNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Language Detection

**Auto-detect language:**

```python
def detect_language(audio_path):
    """Detect language of audio"""
    import whisper
    
    model = whisper.load_model("base")
    
    # Get probabilities for all languages
    result = model.detect_language(audio_path)
    
    language = result["language"]
    confidence = result["language_confidence"]
    
    print(f"Detected: {language} (confidence: {confidence:.2%})")
    
    # Full language mapping
    LANGUAGE_CODE_TO_NAME = {
        "en": "English",
        "es": "Spanish",
        "fr": "French",
        "de": "German",
        "it": "Italian",
        "pt": "Portuguese",
        "zh": "Chinese",
        "ja": "Japanese",
        "ko": "Korean",
        "ar": "Arabic",
        "hi": "Hindi",
        "ur": "Urdu",
    }
    
    print(f"Full name: {LANGUAGE_CODE_TO_NAME.get(language, language)}")
```

## Performance Optimization

**Use smaller quantized models:**

```python
# Load quantized model for faster inference
model = whisper.load_model("tiny")  # 39M parameters

# Use GPU if available
import torch
device = "cuda" if torch.cuda.is_available() else "cpu"
model = model.to(device)

# Transcribe
result = model.transcribe("audio.mp3", device=device)
```

**Streaming Recognition:**

```python
class StreamingRecognizer:
    def __init__(self):
        self.model = whisper.load_model("tiny")
        self.buffer = []
        self.chunk_size = 16000  # 1 second at 16kHz
    
    def process_audio_chunk(self, chunk):
        """Process incoming audio chunks"""
        self.buffer.extend(chunk)
        
        # Process when buffer is full
        if len(self.buffer) >= self.chunk_size:
            audio_segment = np.array(self.buffer[:self.chunk_size])
            
            # Transcribe
            result = self.model.transcribe(audio_segment)
            
            # Remove processed portion
            self.buffer = self.buffer[self.chunk_size:]
            
            return result['text']
        
        return None
```

## Error Handling

**Robust Recognition:**

```python
class RobustRecognizer:
    def __init__(self):
        self.model = whisper.load_model("base")
        self.confidence_threshold = 0.5
    
    def recognize_with_retry(self, audio_path, max_retries=3):
        """Recognize with retry logic"""
        
        for attempt in range(max_retries):
            try:
                result = self.model.transcribe(audio_path)
                
                # Check confidence
                if result.get('confidence', 1.0) > self.confidence_threshold:
                    return result['text']
                else:
                    print(f"Low confidence on attempt {attempt + 1}")
                    
            except Exception as e:
                print(f"Error on attempt {attempt + 1}: {e}")
        
        return None
```

## Key Concepts

- **Frequency**: 16kHz audio recommended
- **Duration**: Whisper works with variable-length audio
- **Noise**: Tolerant to background noise (built-in pre-processing)
- **Languages**: 99 languages with automatic detection
- **GPU**: ~100x faster with CUDA on modern GPUs

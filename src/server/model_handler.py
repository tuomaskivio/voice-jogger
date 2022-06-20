import vosk
from utils_vad import OnnxWrapper
from pathlib import Path
import numpy as np
import json
import time


# Provided by Alexander Veysov
def int2float(sound):
    abs_max = np.abs(sound).max()
    sound = sound.astype('float32')
    if abs_max > 0:
        sound *= 1/abs_max
    sound = sound.squeeze()  # depends on the use case
    return sound

class Recognizer:
    """Handles vosk and vad speech to text"""
    def __init__(self, model_path, sample_rate):
        """Class Constructor"""
        self.vad = OnnxWrapper(str(Path(model_path, 'silero_vad.onnx')))
        self.audio_chunks = []
        self.speech_start_idx = []
        self.start_speech = False
        self.time_since_last_speech = 0.0
        self.speech_end_interval = 0.5
        self.chunk_offset = 3
        self.rate = sample_rate
        
        model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(model, self.rate, json.dumps([
        # System commands
        "start", "stop", "panda", "move", "go", "mode", "distance", "direction", "step", "low", "medium", "high", "size", "tool", "open", "close", "grasp", "rotate",
        "list", "show", "task", "play", "do", "remove", "delete", "save", "home", "finish", "record", "gripper", "position", "spot", "other", "opposite", "counter",
        # Directions
        "up", "down", "left", "right", "forward", "backward", "front", "back",
        # numbers
        "one", "two", "three", "four", "five", "six", "seven", "eight", "nine", "ten", "zero",
        "eleven", "twelve", "thirteen", "fourteen", "fifteen", "sixteen", "seventeen", "eighteen", "nineteen", "twenty",
        "thirty", "forty", "fifty", "sixty", "seventy", "eighty", "ninety", "hundred", "thousand",
        # unknown
        "[unk]"]))
        
    
    def speech_to_text(self, data):
        """Convert speech to text using speech model recognizer"""
        words = []
        # Detect Speech
        self.audio_chunks.append(data)
        audio_int16 = np.frombuffer(data, np.int16);
        audio_float32 = int2float(audio_int16)
        output = self.vad(audio_float32, self.rate)
        if output > 0.5:
            #print("Speec Detected")
            self.start_speech = True
            self.speech_start_idx.append(len(self.audio_chunks) - 1)
            self.time_since_last_speech = time.time()
        else:
            if self.start_speech and time.time() - self.time_since_last_speech > self.speech_end_interval:
                #print("Speech Ended")
                start_idx = np.min(self.speech_start_idx) - self.chunk_offset
                end_idx = np.max(self.speech_start_idx) + self.chunk_offset
                #print(f"Speech Index Interval: [{start_idx}, {end_idx}]")
                self.start_speech = False
                self.speech_start_idx = []
                
                speech = b''.join(self.audio_chunks[start_idx: end_idx])
                # Convert speech to text
                self.rec.AcceptWaveform(speech)
                words = json.loads(self.rec.FinalResult())["text"].split(' ')
        
        return words

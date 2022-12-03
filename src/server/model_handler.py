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
    def __init__(self, model_path, sample_rate, debug_enabled = False):
        """Class Constructor"""
        self.vad = OnnxWrapper(str(Path(model_path, 'silero_vad.onnx')))
        self.audio_buffer = []
        self.start_speech = False
        self.time_since_last_speech = 0.0
        self.speech_end_interval = 0.5
        self.chunk_offset = 8
        self.rate = sample_rate
        self.debug_enabled = debug_enabled
        
        model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(model, self.rate, json.dumps([
        # System commands
        "start", "stop", "panda", "robot", "move", "go", "mode", "distance", "direction", "step", "low", "medium", "high", "size", "tool", "open", "close", "grasp", "rotate",
        "list", "show", "task", "play", "do", "remove", "delete", "save", "home", "finish", "record", "gripper", "position", "spot", "other", "opposite", "counter", "and", "then",
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
        # Detect Speech
        if not self.start_speech:
            self.audio_buffer.append(data)
        audio_int16 = np.frombuffer(data, np.int16)
        audio_float32 = int2float(audio_int16)
        output = self.vad(audio_float32, self.rate)

        if output > 0.5:
            self.debug("Speech Detected")
            self.time_since_last_speech = time.time()
            if not self.start_speech:
                self.start_speech = True
                # Add a few audio chunks before detecting speech
                speech = b''.join(self.audio_buffer[-self.chunk_offset:-1])
                self.rec.AcceptWaveform(speech)
                self.audio_buffer = []

        if self.start_speech:
            self.rec.AcceptWaveform(data)
            partial = json.loads(self.rec.PartialResult())['partial']
            if partial != '':
                self.debug(partial)

            if 'stop' in partial:
                self.rec.Reset()
                words = ['stop','panda']
                return words

        if self.start_speech and time.time() - self.time_since_last_speech > self.speech_end_interval:
            self.start_speech = False
            text = json.loads(self.rec.FinalResult())["text"]
            self.debug('Final result: {text}')
            words = text.split(' ')
            self.debug("Speech Ended")
            return words
        return None

    def debug(self, text):
        if self.debug_enabled:
            print(text)


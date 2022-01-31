import vosk
import json



class Recognizer:
    """Handles vosk speech to text"""
    def __init__(self, model_path, sample_rate):
        """Class Constructor"""
        model = vosk.Model('model')
        self.rec = vosk.KaldiRecognizer(model, sample_rate)
    
    def speech_to_text(self, data):
        """Convert speech to text using speech model recognizer"""
        if self.rec.AcceptWaveform(data): # this is a blocking statement
            words = json.loads(self.rec.Result())["text"].split(' ')
        else:
            words = []
        
        return words
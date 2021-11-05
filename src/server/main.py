import socket
import vosk
import queue
import time
import webrtcvad
import json

from vocab import CMD_ARGS_LOOKUP_TABLE
from vocab import COMMANDS

FORMAT = 'int16'
CHANNELS = 1
RATE = 16000
CHUNK = 1280
IP = "192.168.1.227"
PORT = 50005
MODEL_RESET_TIME = 2 #seconds

# Configs:
# Rate = 16000, CHUNK = 1280
# Rate = 8000, CHUNK = 640

print('Setting up UDP socket')
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp.settimeout(6)
udp.bind((IP, PORT))

# Voice Activity Detector
vad = webrtcvad.Vad()

vad.set_mode(3)

frame_duration = 10  # ms
total_vad_frames = int(RATE * frame_duration / 1000)*2

# Speech Recognizer
model = vosk.Model('model')
rec = vosk.KaldiRecognizer(model, RATE)
prediction = None
partial = None


timer = time.time()
print_reset = False
start_robot = False
try:
    while True:
        data, addr = udp.recvfrom(CHUNK)
        
        
        ################### Voice Activity Detector ######################
        
        data_split = len(data)//total_vad_frames
        
        is_speech = False
        for i in range(data_split):
            data_chunk = data[i*total_vad_frames:(i+1)*total_vad_frames]
            if vad.is_speech(data_chunk, RATE):
                is_speech = True
                break
        
        ##################################################################
            
        if is_speech:
            print_reset = True
        # Recognize speech
            rec.AcceptWaveform(data)
            partial = json.loads(rec.PartialResult())["partial"]
            
            
        if not is_speech or (time.time() - timer) > MODEL_RESET_TIME:
            prediction = partial
            rec.Reset()
            partial = None
            # Reset Timer
            timer = time.time()
            if print_reset:
                print("Model prediction buffer has been reseted!")
                print_reset = False
        
        ################## Speech to text ##########################
        
        if prediction is not None:
            words = prediction.split(' ')
            
            if len(words) > 1:
                cmd_word = words.pop(0)
                get_cmd = COMMANDS.get(CMD_ARGS_LOOKUP_TABLE.get(cmd_word, ''), None)
                if get_cmd is not None:
                    cmd = get_cmd(words)
                    if cmd is not None:
                        # Update timer if command was valid
                        timer = time.time()
                        # Get start and stop command and act accordingly
                        if cmd[0] == 'START':
                            start_robot = True
                            print('Starting with command: ', cmd)
                        elif cmd[0] == 'STOP':
                            start_robot = False
                            print('Stopping with command: ', cmd)
                        
                        if start_robot:
                            # If robot is started, send command to robot here
                            print(cmd)
                else:
                    print('Invalid command word received: ', cmd_word) 
                                            
        
        ############################################################
        
        
except (KeyboardInterrupt, socket.timeout) as e:
    print(e)

print('Shutting down')
udp.close()
# stream.stop()
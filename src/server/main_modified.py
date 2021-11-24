#import socket
import vosk
import queue
import time
import webrtcvad
import json
import sounddevice as sd
import sys
import argparse
import os

from vocab import CMD_ARGS_LOOKUP_TABLE
from vocab import COMMANDS

FORMAT = 'int16'
CHANNELS = 1
RATE = 16000
#CHUNK = 1280
#IP = "192.168.71.43"
#PORT = 50005
MODEL_RESET_TIME = 3 #seconds

# Configs:
# Rate = 16000, CHUNK = 1280
# Rate = 8000, CHUNK = 640

# print('Setting up UDP socket')
# udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udp.settimeout(15)
# udp.bind((IP, PORT))

# Voice Activity Detector
vad = webrtcvad.Vad()

vad.set_mode(3)

frame_duration = 10  # ms
total_vad_frames = int(RATE * frame_duration / 1000)*2

# Speech Recognizer
#model = vosk.Model('model')
#rec = vosk.KaldiRecognizer(model, RATE)
prediction = None
partial = None


timer = time.time()
print_reset = False
start_robot = False

#___________________________________________________________________________________
q = queue.Queue()

def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def callback(indata, frames, time, status):
    """This is called (from a separate thread) for each audio block."""
    if status:
        print(status, file=sys.stderr)
    q.put(bytes(indata))

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument(
    '-l', '--list-devices', action='store_true',
    help='show list of audio devices and exit')
args, remaining = parser.parse_known_args()
if args.list_devices:
    print(sd.query_devices())
    parser.exit(0)
parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter,
    parents=[parser])
parser.add_argument(
    '-f', '--filename', type=str, metavar='FILENAME',
    help='audio file to store recording to')
parser.add_argument(
    '-m', '--model', type=str, metavar='MODEL_PATH',
    help='Path to the model')
parser.add_argument(
    '-d', '--device', type=int_or_str,
    help='input device (numeric ID or substring)')
parser.add_argument(
    '-r', '--samplerate', type=int, help='sampling rate')
args = parser.parse_args(remaining)
#______________________________________________________________________________________



try:
    if args.model is None:
        args.model = "model"
    if not os.path.exists(args.model):
        print ("Please download a model for your language from https://alphacephei.com/vosk/models")
        print ("and unpack as 'model' in the current folder.")
        parser.exit(0)
    if args.samplerate is None:
        device_info = sd.query_devices(args.device, 'input')
        # soundfile expects an int, sounddevice provides a float:
        args.samplerate = int(device_info['default_samplerate'])

    model = vosk.Model(args.model)

    if args.filename:
        dump_fn = open(args.filename, "wb")
    else:
        dump_fn = None



    with sd.RawInputStream(samplerate=16000, blocksize = 8000, device=args.device, dtype='int16', channels=1, callback=callback):

        print('#' * 80)
        print('Press Ctrl+C to stop the recording')
        print('#' * 80)
        
        rec = vosk.KaldiRecognizer(model, 16000)

        while True:
            #print(time.time())
            #data, addr = udp.recvfrom(CHUNK)
            data = q.get()
            
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
                print(words)
                
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
                                prediction = partial
                                rec.Reset()
                                partial = None
                                timer = time.time()
                                if print_reset:
                                    print("Model prediction buffer has been reseted!")
                                    print_reset = False
                    else:
                        print('Invalid command word received: ', cmd_word) 
                                                
            
            ############################################################
        
        
#except (KeyboardInterrupt, socket.timeout) as e:
except (KeyboardInterrupt) as e:
    print(e)

print('Shutting down')
#udp.close()
# stream.stop()

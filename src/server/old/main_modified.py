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
from copy import copy

from commandCreator import CommandCreator

commandCreator = CommandCreator()

# ROS
import rospy
from std_msgs.msg import String

rospy.init_node('text_command_transmitter')
pub = rospy.Publisher("/text_commands", String, queue_size=10) # queue_size gives time for subscriber to process data it gets

#FORMAT = 'int16'
#CHANNELS = 1
RATE = 16000 # 16000 is good for RawInputStream. Higher takes more wrong words. (32000 is the best, 48000 works pretty good)
#CHUNK = 1280
#IP = "192.168.71.43"
#PORT = 50005
#MODEL_RESET_TIME = 2 #seconds

# Configs:
# Rate = 16000, CHUNK = 1280
# Rate = 8000, CHUNK = 640

# print('Setting up UDP socket')
# udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# udp.settimeout(15)
# udp.bind((IP, PORT))

# Voice Activity Detector
#vad = webrtcvad.Vad()
#vad.set_mode(3)

# The WebRTC VAD only accepts 16-bit mono PCM audio, sampled at 8000, 16000, 32000 or 48000 Hz. A frame must be either 10, 20, or 30 ms in duration."
# For example, if your sample rate is 16000 Hz, then the only allowed frame/chunk sizes are 16000 * ({10,20,30} / 1000) = 160, 320 or 480 samples. 
# Since each sample is 2 bytes (16 bits), the only allowed frame/chunk sizes are 320, 640, or 960 bytes.
#frame_duration = 20  # ms
#total_vad_frames = int(RATE * frame_duration / 1000)

# Speech Recognizer
model = vosk.Model('model')
rec = vosk.KaldiRecognizer(model, RATE)
#prediction = None
#partial = None
cmd = None


#timer = time.time()
#print_reset = False
start_robot = False

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


try:

    with sd.RawInputStream(samplerate=RATE, blocksize = 8000, device=None, dtype='int16', channels=1, callback=callback):

        print('')
        print('#' * 80)
        print('Press Ctrl+C to stop the recording')
        print('#' * 80)
        #while True:
        while not rospy.is_shutdown():
            data = q.get()
            # mode: step mode. Faster response time, because in the direction and distance mode distance needs to be recognized.
            if commandCreator.mode == 0:

                if rec.AcceptWaveform(data):
                    rec.Reset()
                else:
                    words = json.loads(rec.PartialResult())["partial"].split(' ')
                    if len(words) > 1:
                        print(words)
                        cmd = commandCreator.getCommand(words)

            # mode: direction and distance
            else:
                if rec.AcceptWaveform(data):
                    words = json.loads(rec.Result())["text"].split(' ')
                    if len(words) > 1:
                        print(words)
                        cmd = commandCreator.getCommand(words)

            if cmd != None:
                #start_robot means start sending commands
                if cmd[0] == 'START':
                    start_robot = True
                    print('Starting with command: ', cmd)
                    rec.Reset()
                    cmd = None

                    # Update RobotMover every time PANDA is started here.
                    pub.publish('MODE ' + commandCreator.mode)
                    pub.publish('STEP SIZE ' + commandCreator.step_size)

                elif cmd[0] == 'STOP':
                    start_robot = False
                    print('Stopping with command: ', cmd)
                    rec.Reset()
                    cmd = None

                # cmds are published after the robot is started
                if start_robot and cmd != None:
                    cmdString = ' '.join(map(str, cmd))
                    pub.publish(cmdString)
                    print(cmdString)
                    rec.Reset()
                    cmd = None

except (KeyboardInterrupt) as e:
    print(e)

print('Shutting down')

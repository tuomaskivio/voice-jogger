import vosk
import queue
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

RATE = 16000 # 16000 is good for RawInputStream. Higher takes more wrong words. (32000 is the best, 48000 works pretty good)

# Speech Recognizer
model = vosk.Model('model')
rec = vosk.KaldiRecognizer(model, RATE)
cmd = None

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
        while not rospy.is_shutdown():
            data = q.get()
            
            if rec.AcceptWaveform(data):
                words = json.loads(rec.Result())["text"].split(' ')
                #if len(words) > 1:
                # print(80*"-")
                # print("All recorded words: ")
                # print(words)
                # print("")
                commandCreator.original_words = words
                cmd = commandCreator.getCommand(True)

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
                    rospy.loginfo("Sent command: " + cmdString)
                    rec.Reset()
                    cmd = None

            while len(commandCreator.current_words) > 0:
                cmd = commandCreator.getCommand(False)

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
                        rospy.loginfo("Sent command: " + cmdString)
                        rec.Reset()
                        cmd = None

            


except (KeyboardInterrupt) as e:
    print(e)

print('Shutting down')

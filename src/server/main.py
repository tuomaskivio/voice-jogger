import queue

from commandCreator import CommandCreator
from udp_handler import UDPReceiver
from model_handler import Recognizer

# Configs:
# Rate = 16000, CHUNK = 1280
# Rate = 8000, CHUNK = 640

RATE = 16000 # 16000 is good for RawInputStream. Higher takes more wrong words. (32000 is the best, 48000 works pretty good)
CHUNK = 1280
PORT = 50005

q = queue.Queue()

# UDP Receiver (Handles Android App comm.)
udp = UDPReceiver(q, CHUNK, "0.0.0.0", PORT)

# Speech Recognizer (Handles speech to text)
rec = Recognizer('model', RATE)
cmd = None
start_robot = False

# Command Creator (Handles words to command logic)
commandCreator = CommandCreator()

# Start udp thread
udp.start()

try:

    while True:
        data = q.get()
        # mode: step mode. Faster response time, because in the direction and distance mode distance needs to be recognized.
        
        #if False:
        words = rec.speech_to_text(data)
        if len(words) > 1:
            print(words)
            cmd = commandCreator.getCommand(words)

        if cmd is not None:
            #start_robot means start sending commands
            if cmd[0] == 'START':
                start_robot = True
                print('Starting with command: ', cmd)

            elif cmd[0] == 'STOP':
                start_robot = False
                print('Stopping with command: ', cmd)

            # cmds are published after the robot is started
            if start_robot and cmd is not None:
                cmdString = ' '.join(map(str, cmd))
                print(cmdString)
            cmd = None

except Exception as e:
    print(e)
finally:
    print('Shutting down...')
    print('Closing UDP thread')
    udp.close_thread = True
    udp.join()
    print('Clearing queue')
    while not q.empty():
        q.get()


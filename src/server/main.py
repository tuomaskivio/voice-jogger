"""Main server code for handling Android->Speech2text->ROS. Use Ctrl-C to stop script"""
import queue

from commandCreator import CommandCreator
from udp_handler import UDPReceiver
from model_handler import Recognizer

# Configs:
# if Rate = 16000, then Chunk = 1280
# if Rate = 8000, then Chunk = 640 (8kHz is not supported with speech model. Can be added later with upsampling before passing to recognizer)

RATE = 16000
CHUNK = 1280
PORT = 50005
ROS_ENABLED = True
DEBUG = True


class Server:

    def __init__(self):
        if ROS_ENABLED:
            # ROS
            import rospy
            from std_msgs.msg import String

            rospy.init_node('text_command_transmitter')
            self.pub = rospy.Publisher("/text_commands", String, queue_size=10) # queue_size gives time for subscriber to process data it gets
            self.pub_priority = rospy.Publisher("/text_commands_priority", String, queue_size=10) # queue_size gives time for subscriber to process data it gets

            # UDP Receiver (Handles Android App comm.)
            q = queue.Queue()
            self.udp = UDPReceiver(q, CHUNK, "0.0.0.0", PORT)

            # Speech Recognizer (Handles speech to text)
            rec = Recognizer('model', RATE, DEBUG)

            # Command Creator (Handles words to command logic)
            self.commandCreator = CommandCreator()

            # Start udp thread
            self.udp.start()

            cmd = None
            self.start_robot = False
            self.is_chain_going = False
            self.previous_and = False
            try:

                while True:
                    # We need this to make this process shutdown when rospy is being used
                    if ROS_ENABLED and rospy.is_shutdown():
                        break
                    if q.empty():
                        continue
                    data = q.get()

                    words = rec.speech_to_text(data)

                    chained_command = False

                    if words != None:
                        # mode: step mode. Faster response time, because in the direction and distance mode, distance needs to be recognized.
                        self.commandCreator.original_words = words
                        cmd = self.commandCreator.getCommand(True)

                    if cmd is not None:
                        #check if there will be another command
                        next_words, is_and, is_end_of_and = self.commandCreator.check_if_chained(words)
                        if next_words is not None:
                            chained_command = True
                            try:
                                cmd.append(is_and)
                                cmd.append(is_end_of_and)
                                self.previous_and = is_and
                            except:
                                pass

                        #start_robot means start sending commands
                        if cmd[0] == 'START':
                            self.start_robot = True
                            print('Starting with command: ', cmd)
                            print(f'Sending configuration to ROS. mode: {self.commandCreator.mode} step_size: {self.commandCreator.step_size}')
                            if ROS_ENABLED:
                                # Send robot configuration to ROS.
                                self.pub.publish('MODE ' + self.commandCreator.mode)
                                self.pub.publish('STEP SIZE ' + self.commandCreator.step_size)

                        elif cmd[0] == 'STOP':
                            print('Sending Command to ROS: STOP')
                            if ROS_ENABLED:
                                self.pub_priority.publish('STOP')
                            self.start_robot = False
                            print('Stopping with command: ', cmd)

                        # cmds are published after the robot is started
                        if self.start_robot and cmd is not None:
                            cmdString = ' '.join(map(str, cmd))
                            print('Sending Command to ROS: ', cmdString)
                            if ROS_ENABLED:
                                self.pub.publish(cmdString)
                            if chained_command:
                                # A new command execution and chaining check
                                self.__check_chained__(next_words)
                        cmd = None

            except Exception as e:
                print('Exception', e)
            finally:
                print('Shutting down...')
                print('Closing UDP thread')
                self.udp.close_thread = True
                self.udp.join()
                print('Clearing queue')
                while not q.empty():
                    q.get()

    def __check_chained__(self, words):
        chained_command = False
        self.commandCreator.original_words = words
        cmd = self.commandCreator.getCommand(True)
        # Get next command words if there is chain word (otherwise None), is chain word AND and
        # is this end of AND command chain.
        next_words, is_and, is_end_of_and = self.commandCreator.check_if_chained(words)
        if next_words is not None:
            chained_command = True
            try:
                # Check if there was AND before this command
                if self.previous_and:
                    cmd.append(True)
                else:
                    cmd.append(is_and)
                cmd.append(is_end_of_and)
                self.previous_and = is_and
            except:
                pass

        else:
            cmd.append(True)
            cmd.append(True)

        if cmd is not None:

            #start_robot means start sending commands
            if cmd[0] == 'START':
                self.start_robot = True
                print('Starting with command: ', cmd)
                print(f'Sending configuration to ROS. mode: {commandCreator.mode} step_size: {commandCreator.step_size}')
                if ROS_ENABLED:
                    # Send robot configuration to ROS.
                    pub.publish('START')
                    pub.publish('MODE ' + commandCreator.mode)
                    pub.publish('STEP SIZE ' + commandCreator.step_size)

            if cmd[0] == 'STOP':
                print('Sending Command to ROS: STOP')
                if ROS_ENABLED:
                    self.pub_priority.publish('STOP')
                self.start_robot = False
                print('Stopping with command: ', cmd)

            # cmds are published after the robot is started
            if self.start_robot and cmd is not None:
                cmdString = ' '.join(map(str, cmd))
                print('Sending Command to ROS: ', cmdString)
                if ROS_ENABLED:
                    self.pub.publish(cmdString)
            if chained_command:
                # A new command execution and chaining check
                self.__check_chained__(next_words)



server = Server()

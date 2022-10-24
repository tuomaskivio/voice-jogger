# voice-jogger
Voice Jogger for Kone533-2021-2022-1 Robotics Project Work

# Server Requirements

This project is designed to work with Ubuntu 20.04 LTS with ROS noetic. It may work with earlier different versions with some tweaks. Following are the main requirements:

| Name             | Version   |
|------------------|-----------|
| Ubuntu           | 20.04 LTS |
| ROS              | Noetic    |
| vosk             | 0.3.42    |
| onnxruntime      | 1.11.1    |
| word2number      | 1.1       |
| rospy            | 1.15.11   |
| numpy            | 1.22.3    |
| actionlib        | 1.13.2    |
| moveit-commander | 1.1.9     |
| franka_gripper   |           |

For a complete list of pypi dependencies, see pip_pkgs.txts

Some dependencies may not be available with PYPI and you may need to get them installed via ROS Workspace. Following are the list of main packages installed in the catkin workspace:

- franka_ros_interface
- geometric_shapes
- moveit
- moveit_msgs
- moveit_resources
- moveit_tutorials
- moveit_visual_tools
- panda_moveit_config
- rviz_visual_tools
- srdfdom

For a complete list of catkin workspace packages, see catkin_pkgs.txt file. You may not need to install all of them if the main packages are available system-wide in /opt/ros/.../lib

For working with real franka panda robot, you will also need to install libfranka. (this should already be installed in Robo lab).

Feel free to provide feedback if some packages are missing and were crucial to install to run this project.


# Usage and Instruction

The code is divided into several parts.

- Android Application: We have a android app that was built on AndroidStudio. The source code is in android directory. Android app reads audio from mobile microphone and sends those raw datagram to UDP sockets in the server.

- Server/main: Server main is responsible for receiving audio signal from mobile app, converting it to text commands, and then publishing those text commands into a ros node topic text_command_transmitter/text_commands. The file to run a server is server/main.py. There are other server implementation in server/microphone_input.py and server/main_modified.py but those can be ignored for now. microphone_input contains experimental commands buffer implementation and it is used to get audio signal from microphone connected directly with computer. This is going to require an additional package sounddevice from pypi. There is a third script called ros_message_transmitter_for_testing_robotMover which can be used for debugging and testing purposes. This will send commands directly to robotmover without the audio signal and speech recognition logic via cli inputs.

- Server/robotomver: Robot mover is responsible for reading text commands from main and sending commands to ROS topics that controls robot manipulation. The file is located at server/robotMover.py

## Running the code

For running the code in simulation:-

In 1st terminal: 

For simulation:

    roslaunch panda_moveit_config demo.launch

For real robot:

    roslaunch panda_moveit_config franka_control.launch robot_ip:=130.230.36.115 load_gripper:=True

In second terminal:

    python3 main.py 
OR

    python3 microphone_input.py 
    
OR

    python3 ros_message_transmitter_for_testing_robotMover.py

In third terminal:

    python3 robotMover.py


## Support Commands and Modes

Following are the most commonly used supported commands. More commands can be found from server/commandCreater.py but the command usage isnt written here

- START PANDA: Once ran, all valid commands are going to be published to ROS topic
- STOP PANDA: Once ran, commands may get recognized but they wont get published
- MODE [STEP, DISTANCE]: Select either step or distance mode
- STEP SIZE [LOW, MID, HIGH]: Select step size. Used in step mode move commands and rotate command
- SAVE POSITION [1,2,3, etc.]: Save current X,Y,Z, RX, RY, RZ position of the  robot tool in the positions.txt file
- POSITION [1,2,3, etc.]: Go to saved position
- RECORD [TASK1, TASK2, etc.]: Start recording task in tasks.txt file
- FINISH: Stop recording task
- PLAY [TASK1, TASK2, etc]: Play recorded task
- HOME: Go to home position of the robot
- REMOVE POSITION [1,2,3, etc.]: Remove position from file
- TOOL OPEN: Open tool
- TOOL CLOSE: Close tool
- TOOL ROTATE: Rotate tool by the step size
- TOOL ROTATE BACK (May not run): Rotate tool in the opposite direction
- TOOL [DISTANCE]: Move tool fingers by given distance


In step:

 - MOVE [UP, DOWN, LEFT, RIGHT, FRONT/FORWARD, BACK/BACKWARD]: Move robot in given direction. In step mode, distance is not provided and the step size is used as distance.

In distance:

- MOVE [UP, DOWN, LEFT, RIGHT, FRONT/FORWARD, BACK/BACKWARD] [DISTANCE]: Similar to step except distance is also provided in millimeters


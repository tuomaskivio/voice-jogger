# Voice Jogger

Voice Jogger for Kone533-2022-2023-1 Robotics Project Work

# System components

## Android Application

We have a android app that was built on AndroidStudio. The source code is in android directory. Android app reads audio from mobile microphone and sends those raw datagram to UDP sockets in the server.

### Wear OS app

IP address, port and sampling rate used by Wear OS app are set from the phone app. The app needs to be open on the watch when editing settings on the phone.

## Speech detection server

*src/server/main.py*

Server main is responsible for receiving audio signal from mobile app, converting it to text commands, and then publishing those text commands into a ros node topic text_command_transmitter/text_commands. The file to run a server is server/main.py. There are other server implementation in server/microphone_input.py and server/main_modified.py but those can be ignored for now. microphone_input contains experimental commands buffer implementation and it is used to get audio signal from microphone connected directly with computer. This is going to require an additional package sounddevice from pypi. There is a third script called ros_message_transmitter_for_testing_robotMover which can be used for debugging and testing purposes. This will send commands directly to robotmover without the audio signal and speech recognition logic via cli inputs.

## Robot mover module

*src/server/robotMover.py*

Robot mover is responsible for reading text commands from main and sending commands to ROS topics that controls robot manipulation. The file is located at server/robotMover.py

# Installation

Feel free to provide feedback if some packages are missing and were crucial to install to run this project.

## Prerequisites

Install ROS Noetic

Follow http://wiki.ros.org/noetic/Installation/Ubuntu for complete instructions
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-catkin-tools
sudo rosdep init
rosdep update
```

## Required ROS packages

```bash
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
sudo apt install ros-noetic-panda-moveit-config
sudo apt install ros-noetic-gazebo-ros-control ros-noetic-rospy-message-converter ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools

sudo apt install ros-${ROS_DISTRO}-libfranka
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools
```

## Python requirements

```bash
sudo pip install -r requirements.txt
```
| Name                    | Version   |
|-------------------------|-----------|
| rospy                   | 1.14.12   |
| rospy_message_converter | 0.5.2     |
| vosk                    | 0.3.31    |
| webrtcvad               | 2.0.10    |
| word2number             | 1.1       |
| sounddevice             | 0.4.3     |

## Building packages

### Simulation

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone -b noetic-devel https://github.com/justagist/panda_simulator
cd panda_simulator
./build_ws.sh # if catkin not found, install catkin tools (apt install python3-catkin-tools)
source devel/setup.bash
pip install -r requirements.txt
```

### Franka Panda

```bash
mkdir -p catkin_ws/src
cd catkin_ws
git clone -b v0.7.1-dev https://github.com/justagist/franka_ros_interface src/franka_ros_interface
catkin build
source devel/setup.bash
```

# Running the code

### Simulation

1. Start gazebo
`roslaunch panda_moveit_config demo_gazebo.launch`
2. Start robotMover
`python robotMover.py`
3. Start python server
`python main.py`

### Franka Panda

1. Start franka panda ros module
`roslaunch panda_moveit_config franka_control.launch robot_ip:=130.230.36.115 load_gripper:=True`
2. Start robotMover
`python robotMover.py`
3. Start python server
`python main.py`

### Debug modes

Instead of starting the `main.py` python server, run one of the following commands
1. Use computer microphone instead of the Android app
`python microphone_input.py`
2. Write commands and bypass speech recognition
`python ros_message_transmitter_for_testing_robotMover.py`

# Usage

## Set up

To use TAKE and GIVE commands the table area has to be defined by saving left-back and front-right corners of the table. And to use DROP command, the drop position has to saved.

1. Hand guide the robot to left-back corner of the table.
    - Save the corner with command: SAVE POSITION CORNER 1
2. Hand guide the robot to right-front corner of the table.
    - Save the corner with command: SAVE POSITION CORNER 2
3. Hand guide the robot to the drop position.
    - Save the position with command: SAVE POSITION DROP
## Supported commands

Commands follow the following structure:

COMMAND KEYWORD parameter *optional parameter*

### Status commands

| Command | Description |
|---------|-------------|
| START PANDA | Start sending commands to robot mover. |
| STOP | Stop sending commands to robot mover. Also stops all current robot movement. |
| RECOVER | Recover robot from invalid status. Required after hand guiding and collisions. |

### Move commands

| Command | Description |
|---------|-------------|
| STOP | Stop all current robot movement. Also stops sending commands to robot mover. |
| MOVE direction *distance*| Move robot to specified direction. Allowed directions are UP, DOWN, FORWARD, BACKWARD, LEFT and RIGHT. If robot is in distance mode, and optional parameter *distance* is given, robot moves the specified distance in millimeters. |
| HOME | Move robot to home position. |
| GRIPPER OPEN | Open gripper fingers. |
| GRIPPER CLOSE | Close gripper fingers. Force limit is 2 N. |
| GRIPPER distance | Move tool fingers to given distance. |
| ROTATE | Rotate gripper clockwise by the step size. |
| ROTATE BACK | Rotate gripper counter-clockwise by the step size. |
| GIVE tool_name | Move specified tool from the saved location to pick-up area. Pick-up area is defined to front-right corner of the table. |
| TAKE tool_name | Move specified tool from the pick-up area to the saved location of the tool. |
| TAKE NEW TOOL | Pick up new tool from pick-up area. Command is followed by the NAME command. |
| NAME tool_name | Save picked tool with the specified name and move the tool to a free location on the table. |
| DROP tool_name | Pick up tool from the saved location and drop it at the saved drop position. |
| DROP ALL | Drop all tools one by one. |
| POSITION position_name | Move robot to the saved position. |

### Modifier commands

| Command | Description |
|---------|-------------|
| AGAIN | Execute previous command again. |
| AND | Used between two or more move commands. Moves the robot diagonally according to the given move commands. |
| THEN | Used between any two or more commands. Executes the commands one after the other. |

### Other commands

| Command | Description |
|---------|-------------|
| SAVE POSITION position_name | Save current robot position with the given name. |
| REMOVE POSITION position_name | Remove the given position. |
| SAVE TOOL tool_name | Save current position as the location for  the given tool. |
| REMOVE TOOL tool_name | Remove the given tool. |
| RECORD task_name | Start recording task with the given name. |
| FINISH | Stop task recording. |
| PLAY task_name | Play recorded task. |
| REMOVE task_name | Remove the given task. |
| MODE STEP/DISTANCE | Change between step and distance modes. In step mode, MOVE commands move the robot by the specified step size. In distance mode move distance in mm can be given. |
| STEP SIZE LOW/MEDIUM/HIGH | Change step size between low, medium and high. Low = 10 mm, Medium = 50 mm, High = 100 mm. |
| SPEED LOW/MEDIUM/HIGH | Change robot speed limit between low, medium and high. Low = 0.05 * max_speed, Medium = 0.2 * max_speed, High = 1 * max_speed. |


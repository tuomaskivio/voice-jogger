# Voice Jogger
Voice Jogger for Kone533-2021-2022-1 Robotics Project Work

# Android App (Alpha)
App will crash at launch (because of microphone permission denied by default in Android). I have not added a menu to grant permission yet so you will need to grant the app permission from Settings -> App -> ...

### Wear OS app
IP address, port and sampling rate used by Wear OS app are set from the phone app. The app needs to be open on the watch when editing settings on the phone.
 
# Install prerequisites
1. Install ROS Noetic

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
3. Franka ROS interface


# Run
### Simulation
1. Start gazebo
`roslaunch panda_moveit_config demo_gazebo.launch`
2. Start robotMover
`python robotMover.py`
3. Start python server
`python main.py`
### Franka Panda
1. Start franka panda ros module
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



1. Required ROS packages
```bash
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
sudo apt install ros-noetic-panda-moveit-config
sudo apt install ros-noetic-gazebo-ros-control ros-noetic-rospy-message-converter ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools

sudo apt install ros-${ROS_DISTRO}-libfranka
sudo apt install ros-$ROS_DISTRO-gazebo-ros-control ros-${ROS_DISTRO}-rospy-message-converter ros-${ROS_DISTRO}-effort-controllers ros-${ROS_DISTRO}-joint-state-controller ros-${ROS_DISTRO}-moveit ros-${ROS_DISTRO}-moveit-commander ros-${ROS_DISTRO}-moveit-visual-tools
```
2. Other
```

```
3. Python requirements
```bash
sudo pip install -r requirements.txt
```
- vosk
- webrtcvad
- word2number
- sounddevice
- onnxruntime
# Building packages
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


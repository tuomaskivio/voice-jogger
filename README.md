# Voice Jogger
Voice Jogger for Kone533-2021-2022-1 Robotics Project Work

# Android App (Alpha)
App will crash at launch (because of microphone permission denied by default in Android). I have not added a menu to grant permission yet so you will need to grant the app permission from Settings -> App -> ...

### Wear OS app
IP address, port and sampling rate used by Wear OS app are set from the phone app. The app needs to be open on the watch when editing settings on the phone.
 
# Prerequisites
1. Required ROS packages
```bash
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
sudo apt install ros-noetic-panda-moveit-config
sudo apt install ros-noetic-gazebo-ros-control ros-noetic-rospy-message-converter ros-noetic-effort-controllers ros-noetic-joint-state-controller ros-noetic-moveit ros-noetic-moveit-commander ros-noetic-moveit-visual-tools
```
2. Python requirements
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
# Run
### Simulation
1. Start gazebo
`roslaunch panda_gazebo panda_world.launch`
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



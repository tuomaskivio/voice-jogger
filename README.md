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



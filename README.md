# Voice Jogger
Voice Jogger for Kone533-2021-2022-1 Robotics Project Work

# Android App (Alpha)
App will crash at launch (because of microphone permission denied by default in Android). I have not added a menu to grant permission yet so you will need to grant the app permission from Settings -> App -> ...

### Wear OS app
IP address, port and sampling rate used by Wear OS app are set from the phone app. The app needs to be open on the watch when editing settings on the phone.
 
# Install
1. Required ROS packages
```
sudo apt install ros-noetic-desktop
sudo apt install ros-noetic-moveit
sudo apt install ros-noetic-libfranka ros-noetic-franka-ros
```
2. Python requirements
```
sudo pip install -r requirements.txt
```
- vosk
- webrtcvad
- word2number
- sounddevice
- onnxruntime
# Run
### Simulation
1. Start rviz
2. Start robotMover
`python src/server/robotMover.py`
3. Start python server
`python src/server/main.py`
### Franka Panda
1. Start franka panda ros module
2. Start robotMover
`python src/server/robotMover.py`
3. Start python server
`python src/server/main.py`
### Debug modes
Instead of starting the `main.py` python server, run 
1. 
2. 



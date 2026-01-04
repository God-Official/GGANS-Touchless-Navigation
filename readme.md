# Gesture-Guided Autonomous Navigation System (GGANS)

A ROS 2–based mobile robotics project enabling touchless, vision-based robot navigation using hand gestures integrated with the Nav2 autonomous navigation stack.
Users can intuitively guide a robot using hand gestures while the robot autonomously plans and executes safe paths in simulation.

## Tech Stack
- ROS 2 Humble Hawksbill
- Ignition Gazebo Fortress
- Nav2 (Navigation2)
- OpenCV
- Mediapipe
- TensorFlow Lite

## System Flow
```
Camera → Gesture Recognition → Navigation Goal / Command
        ↓
     Nav2 Planner → Controller → /cmd_vel → Robot
```

## Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble installed and sourced

## Installation
### Install ROS & Simulation Dependencies
```
    sudo apt update
    sudo apt install -y \
        ros-humble-nav2* \
        ros-humble-ros-ign \
        ros-humble-ros-ign-bridge \
        ros-humble-ros-ign-gazebo \
        ros-humble-cv-bridge \
        ros-humble-image-transport \
        python3-opencv
```
### Install Vision Libraries
```
pip3 install mediapipe tensorflow tensorflow-lite
```

## Workspace Setup
```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

git clone https://github.com/God-Official/GGANS-Touchless-Navigation.git

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the Project
### Launch Gazebo Simulation
```
ros2 launch turtlebot_description gazebo.launch.py
```

### Launch Navigation Stack
```
ros2 launch turtlebot_nav2 nav2.launch.py
```

### Launch Camera Node
```
ros2 launch camera_pub camera_pub.launch.py
```

### Launch Gesture Control
```
ros2 launch gesture_control gesture_control.launch.py
```

### Launch GGANS (Touchless Navigation)
```
ros2 launch touchless_nav touchless_nav.launch.py
```

## Features
- Vision-based, touchless robot control
- Autonomous navigation with obstacle avoidance
- Gesture-driven goal selection
- Fully simulated and reproducible
- Modular ROS 2 design

## Author
### Prateek H A
**Robotics & ROS 2 Developer**  
*Autonomous Navigation | Computer Vision | Human–Robot Interaction***
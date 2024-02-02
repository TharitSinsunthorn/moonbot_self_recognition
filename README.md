# Moonbot software

This repository includes packages for moonbot control and simulation development. Moonbot is the first model modular robot of the Moonshot project. The targets of this robot are self-reconfigurable and self-assembly abilities. We also aims to perform a simulation of moonbot tasks on the lunar surface simulation in Isaac sim. Please look into the repository and give us various recommendations for the furthur development.

## Prerequisties
* Ubuntu 20.04
* ros2-foxy

## Installation
* Installation of ros2-foxy: [https://docs.ros.org/en/foxy/Installation/Alternatives/Ubuntu-Development-Setup.html](), and source the ros2 package in the terminal.
```bash
source /opt/ros/foxy/setup.bash
```

* Install dependencies
```bash
sudo apt-get update

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install ros-foxy-xacro

sudo apt-get install ros-foxy-ros2-control 

sudo apt-get install ros-foxy-ros2-controllers

sudo apt-get install ros-foxy-gazebo-ros2-control

sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk*

sudo apt-get install ros-$ROS_DISTRO-dynamixel-workbench*

sudo apt-get install joystick jstest

sudo apt-get install ros-foxy-joy* ros-foxy-teleop*

```
* Install program
```bash
cd ros2_ws/

colcon build --symlink-install

source install/setup.bash
```

## Usage of Moonbot simulation and control
To connect the moonbot and demonstrate the simple gait motion
* Simulation in Gazebo
```bash
## terminal 1
ros2 launch moonbot_gazebo spawn_moonbot.launch.py
## terminal 2: For walking demo
ros2 run moonbot_gazebo sim_pub.py
```
<p align="center">
  <img src="https://github.com/TharitSinsunthorn/noppakorn-test/blob/develop/moonbot_gazebo.png" alt="Moonbot's Gazebo">
</p>


## Connect to real robot

* Separately connect to leg modules
```bash
## terminal 1
ros2 launch dynamixel_hardware LF.py
## terminal 2
ros2 laucnh dynamixel_hardware RF.py
## terminal 3
ros2 launch dynamixel_hardware LR.py
## terminal 4
ros2 launch dynamixel_hardware RR.py
```
```bash
## terminal 2
## For walking demo: CRAWL GAIT (default)
ros2 run moonbot_control real_pub.py 
```
<p align="center">
  <img src="https://github.com/TharitSinsunthorn/noppakorn-test/blob/develop/crawl_gait2.gif" alt="Moonbot's crawl gait">
</p>

```bash
## terminal 2
## For walking demo: TROT GAIT
ros2 run moonbot_control real_pub.py --ros-args -p gait_type:="trot" 
```
<p align="center">
  <img src="https://github.com/TharitSinsunthorn/noppakorn-test/blob/develop/trot2.gif" alt="Moonbot's trot gait">
</p>



* Self-recognition test
```bash 
## terminal 1: module detection
ros2 launch moonbot_control modular_detection.launch.py
```

```bash
## terminal 2: modular locomotion
ros2 run moonbot_control modular.py
```

## Author
Tharit Sinsunthorn

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

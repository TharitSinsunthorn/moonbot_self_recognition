# Moonbot software

This repository includes packages for moonbot control and simulation development. Moonbot is the first model modular robot of the Moonshot project. The targets of this robot are self-reconfigurable and self-assembly abilities. We also aims to perform a simulation of moonbot tasks on the lunar surface simulation in Isaac sim. Please look into the repository and give us various recommendations for the furthur development.

## Prerequisties
* Ubuntu 22.04
* ros2-humble

## Installation
* Installation of ros2-humblee: [https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html](), and source the ros2 package in the terminal.
```bash
source /opt/ros/humble/setup.bash
```

* Install dependencies
```bash
sudo apt-get update

rosdep install --from-paths src --ignore-src -r -y

sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk*

sudo apt-get install ros-$ROS_DISTRO-dynamixel-workbench*

sudo apt-get install joystick

sudo apt-get install ros-humble-joy* ros-humble-teleop*

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


* Self-recognition test
```bash 
## terminal 1: module detection
ros2 launch moonbot_control modular_detection.launch.py
```

```bash
## terminal 2: modular locomotion
ros2 run moonbot_control modular.py
```

<p align="center">
  <img src="https://github.com/TharitSinsunthorn/noppakorn-test/blob/develop/selfrecog.gif" alt="Moonbot's self-recognition">

  <img src="https://github.com/TharitSinsunthorn/noppakorn-test/blob/develop/selfrecog2.gif" alt="Moonbot's self-recognition2">
</p>


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



## Packages description 
### moonbot_description
- URDF and mesh files for moonbot.
- RVIZ configuration.
- Launch file for visualizing the moonbot model in RVIZ

```bash 
# For default model
ros2 launch moonbot_description urdf_visualize.launch.py

# For changing robot model
ros2 launch moonbot_description urdf_visualize urdf_file:=your_robot.urdf
```

### moonbot_gazebo
- Model and world files for Gazebo simulation. 
- Launch file to spawn robot in Gazebo

World files and robot model are also changable by the following commands
```bash
# For changing robot model
ros2 launch moonbot_gazebo spawn_moonbot.launch.py urdf_file:=your_robot.urdf 

# For changing world file (The example for lunar surface map)
ros2 launch moonbot_gazebo spawn_moonbot.launch.py world_file_name:=moonbot_box.world
```

### moonbot_custom_interfaces
- custom **msg** and **srv** for ros2 communication in Moonbot

### moonbot_control
- Control scripts for moonbot's locomotion 
- Modular connection scripts
- Launch files for loading ros2 _control and controllers
- Config files for ros2 controllers (for simulation)

### moonbot_camera
- Package for the future work of implementation of depth camera on the Moonbot.

### dynamixel_hardware
- ros2_control hardware interface for Dynamixels. 
- Config files for controllers of each leg.
- Launch files to connect to the real robot

### moonbot_teleop
- Joy stick conteller configuration
- Launch file and script setting for Moonbot teleoperation.


## Author
Tharit Sinsunthorn
Danish AI
Pascal Pama

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

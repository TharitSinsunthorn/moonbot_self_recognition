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
rosdep install --from-paths src --ignore-src -r -y
```
Install program
```bash
cd ros2_ws/src

colcon build --symlink-install

source install/setup.bash
```

## Usage of Moonbot simulation and control
To connect the moonbot and demonstrate the simple gait motion
```bash
## terminal 1
ros2 launch dynamixel_hardware bring_up_on_hardware.launch.py
```
```bash
## terminal 2
## For standing up the robot
ros2 run moonbot_gazebo action_joint.py
## For walking demo
ros2 run moonbot_gazebo limb_joint.py
```

<img src="moonbot.png">

## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

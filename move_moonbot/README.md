## move_moonbot (ROS package)

This is the ROS2 package used to move the moonbot in four cardinal directions. The package uses gait patterns generated and optimized using Genetic Algorithm ()[Colab Notebook].

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


## Usage of moonbot package

### Nodes

- keyboard\_controller: This node reads the keypress and publishes the desired movement to *cmd\_bot* ros topic.
- move\_robot: This node reads to *cmd\_bot* ros topic and calls the client-service to move the limbs needed for the movement in required sequence. This act as the centralized node for controlling each limb for moving the center body.
- joint\_interface: This node act as interface between move\_robot and each limb. It defines the service for moving a limb. There are 4 of this node, each for each limb. Furthermore, the node also does minimum jerk trajectory generation and publishes the angles for each servos to  *target\_joint\_angles\_l{limb\_num}*
- dynamixel\_controller: This node act as an interface between dynamixel hardware and software, sending commands to each servos.


### utilities

- limb\_kinematics: defines the inverse and forward kinematics for each limb
- params: parameter for calibration of software module
- saved\_sequences: stores the gait patterns for movements
- utilities: utility functions

### Launch File

The launch file launched all the nodes considering all the limbs:
```
ros2 launch moonbot moonbot_launch.py
```


## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

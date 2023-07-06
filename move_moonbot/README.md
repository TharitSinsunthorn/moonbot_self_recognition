## move_moonbot (ROS package)

This is the ros package designed to move the moonbot in any direction we want.

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

There are 4 node executable in this package:
- body\_controller: To control the body of the robot.
- joint\_controller: 4 nodes based on this executable is created. This does the inverse kinematics to give the angles based on the tip position.
- joint\_interface: 4 nodes based on this executable is create. This acts as as intermediate between joint\_controller node and dynamixel\_control.
- dynamixel\_control: Acts as interface between ros and servos. 

The launch file launched all the nodes considering all the limbs:
```
ros2 launch moonbot moonbot_launch.py
```


## Contributing

Pull requests are welcome. For major changes, please open an issue first
to discuss what you would like to change.

Please make sure to update tests as appropriate.

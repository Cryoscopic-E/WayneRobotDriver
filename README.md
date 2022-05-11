# Wayne Soft Robot Drivers

PlatformIO project for VScode.

This project contains the arduino driver code for wayne and the ROS interface.



Required to install: 
- [Ros Noetic](http://wiki.ros.org/noetic)
- [rosserial](http://wiki.ros.org/rosserial)

## Clone the repo:

` git clone https://github.com/Cryoscopic-E/WayneRobotDriver `


## ROS Setup:

`cd WayneRobotDriver/ros_wayne_ws `

`catkin_make`

`source ./devel/setup.bash`


## Arduino Setup:

`cd WayneRobotDriver/lib`

`rosrun rosserial_arduino make_libraries.py .`


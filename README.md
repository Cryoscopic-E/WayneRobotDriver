# Wayne Soft Robot Drivers

PlatformIO project for VScode.

This project contains the arduino driver code for wayne and the ROS interface.

Required to install: [rosserial](http://wiki.ros.org/rosserial)

## Clone the repo:

` git clone https://github.com/Cryoscopic-E/WayneRobotDriver `


## Arduino Setup:

` cd WayneRobotDriver/lib`

`rosrun rosserial_arduino make_libraries.py .`

## ROS Setup:

`cd ros_wayne_ws `

`catkin_make`

`source ./devel/setup.bash`
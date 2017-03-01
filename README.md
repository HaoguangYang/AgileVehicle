# AgileVehicle
## The AgileVehicle Project, an automated road vehicle that take you wherever your destination is in whatever attitude you want.
Coming soon.

##Linux Prerequisites
For Ubuntu and other Debian based users, please run the following command to ensure everything is set up:

sudo apt-get install g++ arduino libsdl2-dev joystick

##Directory Structure

####/SteeringWheelControl
Controlling the suspnsion-motor assembly with Logitech G29 and Arduino. Added force feedback but not tested.

####/Arduino
The Arduino code for the function above.

####/ROS
ROS implementation of the system. Catkin workspace.

**IMPORTANT** Please properly setup ROS (version: kinetic kame), please refer to official documents and tutorials at:

[中文](http://wiki.ros.org/cn/ROS/Tutorials)

[English](http://wiki.ros.org/ROS/Tutorials)


#####/ROS/Arduino
ROS based Arduino code able to transmit data at 30Hz.

#####/ROS/src
Home to ROS packages and services. Currently including:

>/ROS/src/steering_wheel: Steering Wheel Control Utilities which should read Logitech G29 data and publish it using custom message prescribed in `msg/joyinfoex.msg` under topic `WheelControl` (**DONE**), or in One-Wheel-Debug mode, publish directly to `WheelControl` topic which is an `Int32MultiArray` and directly received by Arduino, see Arduino code for reference (**UNDONE**).

>/ROS/src/dyna_core: dynamic/kinematic control algorithms which subscribes the data from Arduino at four wheels and topic `WheelControl`, calculates the state of the vehicle, and publish data under `WheelControl` topic. NOT INITIALLIZED YET.

>Future packages: setup module which should be breaken away from steering_wheel, and vision navigation packages.


####/libelas
Integration of OpenCV in libelas, merged libelas-gpu to implement CUDA, merged robotology/stereo-vision and working on migration from yarp to ROS interface. Untested.
> Reference:
> https://github.com/goldbattle/libelas-gpu
> https://github.com/robotology/stereo-vision

####/LogitechFFDrivers
Logitech G29 drivers source and interface for force feedback.


####/Motor Control
Arduino code for motor control on Patroller, the miniaturized test platform built for Mr. Jianhui Zhao.


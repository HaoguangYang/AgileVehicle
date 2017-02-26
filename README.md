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
ROS implementation of the system.

####/joystick_utils
Joystick API library

####/libelas
Integration of OpenCV in libelas, merged libelas-gpu to implement CUDA, merged robotology/stereo-vision and working on migration from yarp to ROS interface. Untested.
> Reference:
> https://github.com/goldbattle/libelas-gpu
> https://github.com/robotology/stereo-vision

####/LogitechFFDrivers
Logitech G29 drivers source and interface for force feedback.

####/DynaCore
Possible dynamic control interface. (DRAFT)

####/Motor Control
Arduino code for motor control on Patroller, the miniaturized test platform built with Mr. Jianhui Zhao.

###Notice for the holiday
Workstation was down for the holiday, commits between Jan.18 and Feb.14 are made in Windows environment, therefore Linux-specific and ROS related codes are un-verified DRAFT. Could be buggy XD.

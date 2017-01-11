# AgileVehicle
## The AgileVehicle Project, an automated road vehicle that take you wherever your destination is in whatever attitude you want.
Coming soon.

##Directory Structure

####/joystick_status
includes linux joystick utilities sources.
/joystick_status/TControl
Current working directory, adding serial interfaces that can run on linux. See reference folder for a RS232 example.
current TControl requires SDL (libsdl-dev) installed. You can also refer to ./utils/jstest.c for another approach.

Update on Jan 11:
Seems a little bit troublesome to use SDL, may refer to joystick library and use ioctl directly...

####/libelas_opencv_test
Integration of OpenCV in libelas, slow on progress and untested.

####/LogitechFFDrivers
Logitech G29 drivers source and interface for force feedback.

####/Motor Control
Arduino code for motor control on Patroller, the miniaturized test platform built with Mr. Jianhui Zhao.

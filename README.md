# AgileVehicle
## The AgileVehicle Project, an automated road vehicle that take you wherever your destination is in whatever attitude you want.

## Latest Update:
**Mar 29** Won Silver Medal on the 45th International Exhibition of Inventions of Geneva.

**Mar 25** Transversal maneuvers realized through open-loop control only.

## Linux Prerequisites
For Ubuntu and other Debian based users, please run the following command to ensure everything is set up:

```sh
sudo apt-get install g++ arduino libsdl2-dev
```

## Directory Structure

#### /without_ROS.old
Deprecated files containing no ROS modules, including the old steering wheel and Arduino utilities.

#### /ROS
ROS implementation of the system. Catkin workspace.

**IMPORTANT** Please properly setup ROS (version: kinetic kame), please refer to official documents and tutorials at:

[中文](http://wiki.ros.org/cn/ROS/Tutorials)
[English](http://wiki.ros.org/ROS/Tutorials)

Additional dependencies:

```sh
sudo apt-get install ros-kinetic-ecl-ipc ros-kinetic-rosserial-arduino ros-kinetic-rosserial
```

##### /ROS/Arduino
ROS based Arduino code able to transmit data at 30Hz. Run `./bootstrap` to prepare the system.

##### /ROS/src
Home to ROS packages and services. Currently including:

> /ROS/src/steering_wheel: Steering Wheel Control Utilities which should read Logitech G29 data and publish it using custom message prescribed in `msg/joyinfoex.msg` under topic `WheelControl`, or in a situation where no steering wheel is found, uses keyboard as input device. ALL BUT force feedback are **DONE**.

> /ROS/src/agile_v_core: Core of the Agile-V system, including communication handling, kinematic/dynamic controllers, GUI, etc.

> /ROS/src/virtual_device_debugger: Simulator for the Agile-V system.

> Future packages: setup module which should be breaken away from steering_wheel, and vision navigation packages.

> /ROS/src/agile_v_vision: Integration of OpenCV in libelas, merged libelas-gpu to implement CUDA, merged robotology/stereo-vision and working on migration from yarp to ROS interface. currently in , untested.
> Reference:
> 
> https://github.com/goldbattle/libelas-gpu
> https://github.com/robotology/stereo-vision

#### /LogitechFFDrivers
Logitech G29 drivers source and interface for force feedback.


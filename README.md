# AgileVehicle
## The AgileVehicle Project, an automated road vehicle that take you wherever your destination is in whatever attitude you want.

## Latest Update:
**Mar 29** Won Silver Medal on the 45th International Exhibition of Inventions of Geneva.

**Mar 25** Transversal maneuvers realized through open-loop control only.

## Linux Prerequisites
For Ubuntu and other Debian based users, please run the following command to ensure everything is set up:

```sh
sudo apt-get install g++ arduino libsdl2-dev
# Setup ROS
sudo sh -c 'echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
apt-cache search ros-kinetic
sudo rosdep init
rosdep update
sudo echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
# Additional Dependencies
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt-get install ros-kinetic-ecl-ipc ros-kinetic-rosserial-arduino ros-kinetic-rosserial rosbash ros-kinetic-rospack ros-kinetic-cv_bridge ros-kinetic-image-transport
sudo apt-get install libirrlicht-dev libglfw3 libglew-dev slurm swig libglm-dev #for Virtual Device Debugger.
```

## Directory Structure

#### /without_ROS.old
Deprecated files containing no ROS modules, including the old steering wheel and Arduino utilities.

#### /ROS
ROS implementation of the system. Catkin workspace.

**IMPORTANT** Please properly setup ROS (version: kinetic kame), please refer to official documents and tutorials at:

[中文](http://wiki.ros.org/cn/ROS/Tutorials)
[English](http://wiki.ros.org/ROS/Tutorials)


##### /ROS/Arduino
ROS based Arduino code able to transmit data at 30Hz. Run `./bootstrap.sh` (or the following command) to prepare the system.

```sh
sudo apt-get install ros-kinetic-rosserial-arduino
sudo apt-get install ros-kinetic-rosserial
sudo chmod 777 /usr/share/arduino/libraries/
cd /usr/share/arduino/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

##### /ROS/src
Home to ROS packages and services. Currently including:

> /ROS/src/steering_wheel: Steering Wheel Control Utilities which should read Logitech G29 data and publish it using custom message prescribed in `msg/joyinfoex.msg` under topic `WheelControl`, or in a situation where no steering wheel is found, uses keyboard as input device. ALL BUT force feedback are **DONE**.

> /ROS/src/agile_v_core: Core of the Agile-V system, including communication handling, kinematic/dynamic controllers, GUI, etc.

> /ROS/src/virtual_device_debugger: Simulator for the Agile-V system. Rely on Chrono Engine, Please compile and install Chrono with core, irrlicht, and parallel as suggested on [This Page](http://api.projectchrono.org/tutorial_install_chrono.html). Otherwise, please delete this folder before compilation.

> Future packages: setup module which should be breaken away from steering_wheel, and vision navigation packages.

> /ROS/src/agile_v_vision: Integration of OpenCV in libelas, merged libelas-gpu to implement CUDA, merged robotology/stereo-vision and working on migration from yarp to ROS interface. currently in , untested.
> Reference:
> 
> https://github.com/goldbattle/libelas-gpu
> https://github.com/robotology/stereo-vision

#### /LogitechFFDrivers
Logitech G29 drivers source and interface for force feedback.


## Compilation
run `./make.sh` from this directory, and enter the USB address of Arduinos following the instructions on the screen.

## Run the Program
run `./run.sh` from this directory. If the systems detects Arduino's presence it will run the control program, otherwise it will run the virtual_device_debugger, which is unavailble without Chrono installed.


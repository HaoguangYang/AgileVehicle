#ifndef STEERING_WHEEL_H
#define STEERING_WHEEL_H

#include <stdio.h>
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <bitset>
#include "ros/ros.h"
#include "steering_wheel/joyinfoex.h"
#include "std_msgs/UInt16MultiArray.h"
//#include "std_msgs/Int32MultiArray.h"

#include <SDL2/SDL.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

int setup(void);
/*****************************
setting up driving devices, list devices for selection if multiple devices found.
*****************************/

int FFupdate(SDL_Joystick * joystick , unsigned short center);
/*****************************
Fource Feedback update
Available for certain steering wheel hardwares and drivers, else return -1
*****************************/

void ActuaterFeedback(const std_msgs::UInt16MultiArray& ActuatorStatus);
/*****************************
Update the screen once Arduino returns a number.
ONLY FOR ONE WHEEL DEBUGGING USE!
*****************************/

int main(int argc, char* argv[]);
/*****************************
Read the steering wheel
and publish data to system under topic "SteeringWheel"
or directly to Arduino (ONE WHEEL DEBUGGING USE ONLY)
*****************************/

#endif

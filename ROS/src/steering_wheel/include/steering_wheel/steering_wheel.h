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

int FFupdate(SDL_Joystick * joystick , unsigned short center);

void ActuaterFeedback(const std_msgs::UInt16MultiArray& ActuatorStatus);

int main(int argc, char* argv[]);

#endif
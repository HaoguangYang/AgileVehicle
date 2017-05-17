#ifndef GUI_H
#define GUI_H

#include "globals.h"
#include <SDL2/SDL.h>
#include "subscribers.h"
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "agile_v_core/joyinfoex.h"

int GUI_Init(void);

void GUIUpdateInput(const agile_v_core::joyinfoex& joyinfo);

SDL_Point* pointsRotTrans(int n, SDL_Point *actual, float angle, bool isRAD, int *Trans);

void DrawWheel0(const std_msgs::UInt16MultiArray& WheelStatus0);

void DrawWheel1(const std_msgs::UInt16MultiArray& WheelStatus1);

void DrawWheel2(const std_msgs::UInt16MultiArray& WheelStatus2);

void DrawWheel3(const std_msgs::UInt16MultiArray& WheelStatus3);

void SDL_Cleanup(void);

#endif

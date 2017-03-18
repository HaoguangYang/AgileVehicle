#ifndef END_PUBLISHER_H
#define END_PUBLISHER_H

#include "ros/ros.h"
#include "encoder.h"
#include "kinematicCtrl.h"
#include "std_msgs/UInt16MultiArray.h"

extern uint16_t reverse_MotorPerformance(double Speed, double Torque);

extern int publishToWheels(double* steerVal, double* driveVal, double* Torque);

#endif

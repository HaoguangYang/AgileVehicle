#ifndef END_PUBLISHER_H
#define END_PUBLISHER_H

#include "ros/ros.h"
#include "encoder.h"
#include "kinematicCtrl.h"
#include "std_msgs/UInt16MultiArray.h"
#include "agile_v_core/electric.h"
#include "agile_v_core/kinematics.h"

int reverse_MotorPerformance(double Speed, double Torque);
/***********************************
Given current motor speed and / or Torque desired for a BLDC (Brushless DC) motor
Generate BLDC Driver input signal value (0~255)
***********************************/

void publishToWheels(ros::NodeHandle handle, ros::Publisher* wheel_pub, std_msgs::UInt16MultiArray* WheelCtrl, \
                     double* steerVal, double* driveVal, double* Torque);
/***********************************
Publish data to Arduinos the driving and steering data (0~255, 0~4096)
***********************************/

void publishToUser(ros::NodeHandle handle, ros::Publisher kineStat);
/***********************************
Publish data to User Interface, including Power Status, Wheel status, etc.
***********************************/

#endif

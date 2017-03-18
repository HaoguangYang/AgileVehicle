#ifndef SUBSCRIBERS_H
#define SUBSCRIBERS_H

#include "kinematicCtrl.h"
#include "encoder.h"
#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

void readFromWheelsDrv00(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv01(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv02(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv03(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsPwr00(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr01(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr02(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr03(const std_msgs::Float32MultiArray& powerData);

#endif


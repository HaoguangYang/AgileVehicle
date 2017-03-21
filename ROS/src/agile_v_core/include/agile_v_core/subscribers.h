#ifndef SUBSCRIBERS_H
#define SUBSCRIBERS_H

#include "kinematicCtrl.h"
#include "encoder.h"
#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "globals.h"

//Read encoder data from Arduinos
void readFromWheelsDrv00(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv01(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv02(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv03(const std_msgs::UInt16MultiArray& wheelData);

//Initialize Encoders at startup
void encodersInit(int i, int j, uint16_t zero);

//Read power data from Arduinos
void readFromWheelsPwr00(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr01(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr02(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr03(const std_msgs::Float32MultiArray& powerData);

#endif


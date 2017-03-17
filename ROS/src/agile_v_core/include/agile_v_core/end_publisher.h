#ifndef END_PUBLISHER_H
#define END_PUBLISHER_H

#include "ros/ros.h"
#include "encoder.h"
#include "kinematicCtrl.h"

ros::Publisher wheel_pub[4];
wheel_pub[0] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl00",2);
wheel_pub[1] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl01",2);
wheel_pub[2] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl02",2);
wheel_pub[3] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl03",2);

uint16_t reverse_MotorPerformance(double Speed, double Torque);

int main(double* steerVal, double* driveVal, double* Torque);

#endif

#ifndef SUBSCRIBERS_H
#define SUBSCRIBERS_H

#include "kinematicCtrl.h"
#include "encoder.h"
#include "ros/ros.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

class ElectricStat{
public:
	float _volt; // 固连车辆坐标系，xy
	float _ampS;
	float _ampD;
	
	float getUnitPwr(){ return (_volt*(_ampS+_ampD));}
	
	float getUnitCur(){	return (_ampS + _ampD);}
	
	float getUnitVolt(){ return (_volt);}
};

extern ElectricStat ElectricMon[4];

void readFromWheelsDrv00(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv01(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv02(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsDrv03(const std_msgs::UInt16MultiArray& wheelData);
void readFromWheelsPwr00(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr01(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr02(const std_msgs::Float32MultiArray& powerData);
void readFromWheelsPwr03(const std_msgs::Float32MultiArray& powerData);

#endif


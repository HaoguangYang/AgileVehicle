#ifndef KINEMATICCTRL_H
#define KINEMATICCTRL_H

#include <stdlib.h>
#include "encoder.h"

typedef struct VehiclePhysicalParams {
	double TrackWidth; // 左右轮间距
	double WheelBase;  // 前后轴距
	double Mass;       // 质量
	double WheelRadius; // 轮子半径
};
typedef struct SensorParams{
	int encoderRes;
	//bla bla bla...
};
typedef struct Kinematic{
	double speed[2]; // 固连车辆坐标系，xy
	double omega;
	double accel[2];
	double angularAccel;
};
VehiclePhysicalParams Vehicle;
Kinematic Actual;

// 左前，右前，左后，右后
double steerVal[4]={0}; // 四个轮子转弯角度
double driveVal[4]={0}; // 四个轮子线速度
double Torque[4]={0}; // 四个轮子扭矩

VehiclePhysicalParams GetVehicleData(void);

double SteeringWheel2Radius (int SteeringWheelVal, int mode);

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal);

int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal, double* Correction);

#endif

#ifndef KINEMATICCTRL_H
#define KINEMATICCTRL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
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

extern double step_time;            //Arduino refresh time

extern Encoder Enc[2][4];           //[0][*] - Steering; [1][*] - Driving
extern Kinematic Actual;
extern double steerVal[4];          // 四个轮子转弯角度
extern double driveVal[4];          // 四个轮子线速度
extern double Torque[4];            // 四个轮子扭矩
// 左前，右前，左后，右后
extern VehiclePhysicalParams Vehicle;

void GetVehicleData(int argc, char* argv[]);

double SteeringWheel2Radius (int SteeringWheelVal, int mode);

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal);

int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal);

#endif

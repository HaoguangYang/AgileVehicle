#ifndef KINEMATICCTRL_H
#define KINEMATICCTRL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "encoder.h"

typedef struct VehiclePhysicalParams {
	double TrackWidth;
	double WheelBase;
	double Mass;
	double WheelRadius;
};
typedef struct SensorParams{
	int encoderRes;
	//bla bla bla...
};
typedef struct Kinematic{
	double speed[2];
	double omega;
	double accel[2];
	double angularAccel;
};

extern double step_time;            //Arduino refresh time

extern Encoder Enc[2][4];          //[0][*] - Steering; [1][*] - Driving
extern Kinematic Actual;
extern double steerVal[4];
extern double driveVal[4];
extern double Torque[4];

void GetVehicleData(int argc, char* argv[]);

extern double SteeringWheel2Radius (int SteeringWheelVal, int mode);

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal);

extern int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal);

#endif

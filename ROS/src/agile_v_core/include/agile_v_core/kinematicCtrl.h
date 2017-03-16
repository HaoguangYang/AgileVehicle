#ifndef KINEMATICCTRL_H
#define KINEMATICCTRL_H

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
VehiclePhysicalParams Vehicle;
Kinematic Actual;

VehiclePhysicalParams GetVehicleData(void);

double SteeringWheel2Radius (int SteeringWheelVal, int mode);

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal);

int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal, double* Correction);

#endif

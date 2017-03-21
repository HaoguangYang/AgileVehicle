#ifndef KINEMATICCTRL_H
#define KINEMATICCTRL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "globals.h"

void GetVehicleData(int argc, char* argv[]);

double SteeringWheel2Radius (int SteeringWheelVal, int mode);

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal);

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal);

int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal);

#endif

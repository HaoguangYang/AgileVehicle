#ifndef KINEMATICCTRL_H
#define KINEMATICCTRL_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "encoder.h"
#include "globals.h"

void GetVehicleData(int argc, char* argv[]);
//Assess vehicle physical data: mass, size, etc.

double SteeringWheel2Radius (int SteeringWheelVal, int mode);
//Remapping of steering wheel input to steering radius of the vehicle.

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal);
//Kinematic Open Loop Centralized Steering

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal);
//Kinematic Closed Loop Centralized Steering

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal);
//Kinematic Closed Loop Heading-locked Steering

int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal);
//Kinematic Controller Core

#endif

//#include <limits.h>
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

VehiclePhysicalParams GetVehicleData()
{
	
}

double SteeringWheel2Radius (int SteeringWheelVal, int mode)
{
	switch (mode)
	case (0)			//Default is using a y=kx+1/x model to cast -32767~32767 to -inf~inf
	{
		return (1/SteeringWheelVal-SteeringWheelVal/32767*32767);
		break;
	}
	case (1)			//Using y=k/[x(+-)b] model
	{
		if (SteeringWheelVal>=0)
			return (1/SteeringWheelVal-1/32767);
		else
			return (1/SteeringWheelVal+1/32767);
		break;
	}
}

uint16_t reverse_MotorPerformance(double Speed, double Torque)
{
    
}

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal)
{
    //Kinematic Open-Loop Centralized Steering Controller
	if (fabs(radius)>=100000 && fabs(speed/radius)<=0.0001)				//filtering out excessively slow turns
	{
		for (int i=0;i<4;i++)
			steerVal[i] = 0;
			driveVal[i] = speed;
	}
	else
	{
		double leftBase = radius-Vehicle.TrackWidth/2;
		double rightBase = radius+Vehicle.TrackWidth/2;
		steerVal[0] = Encoder::reverseAngleLookup(atan(Vehicle.WheelBase/2/leftBase));
		steerVal[1] = Encoder::reverseAngleLookup(atan(Vehicle.WheelBase/2/rightBase));
		steerVal[2] = -steerVal[0];
		steerVal[3] = -steerVal[1];
		double omega = speed/radius;
		double Rleft = sqrt(leftBase*leftBase+Vehicle.WheelBase*Vehicle.WheelBase/4);
		double Rright = sqrt(rightBase*rightBase+Vehicle.WheelBase*Vehicle.WheelBase/4);
		driveVal[0] = omega*Rleft;
		driveVal[1] = omega*Rright;
		driveVal[2] = driveVal[0];
		driveVal[3] = driveVal[1];
	}
	//steerVal NEEDS CONVERSION!!!
	//driveVal NEEDS CONVERSION!!!
	return;
}

void KCLCSteering(double radius, double speed, double* steerVal, double* driveVal)
{
    //Kinematic Closed-Loop Centralized Steering Controller
    Kinematic Target;
    Target.speed [1] = 0;
    Target.speed [0] = speed;
	if (fabs(radius)>=100000 && fabs(speed/radius)<=0.0001)				//filtering out excessively slow turns
	{
		Target.omega = 0;
	}
	else
	{
		Target.omega = speed/radius;
	}
	int errnum = Controller(Target, steerActual, driveActual, steerVal, driveVal);
	//steerVal NEEDS CONVERSION!!!
	//driveVal NEEDS CONVERSION!!!
	return;
}

void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal)
{
    //Kinematic Closed-Loop Heading-locked Steering
    Kinematic Target;
    Target.speed [1] = speed*sin(steering_wheel_input/32767*pi/2);
    Target.speed [0] = speed*cos(steering_wheel_input/32767*pi/2);
    Target.omega = 0;
    
    int errnum = Controller(Target, steerActual, driveActual, steerVal, driveVal);
    return;
}

int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal)
{
	double u[2][4];
	//double v[4];
	Kinematic Error;
	
	#pragma omp parallel for num_threads(4)
	for (int i=0;i<4;i++)
	{
		u[0][i] = driveActual[i]*cos(steerActual[i]);
		u[1][i] = driveActual[i]*sin(steerActual[i]);
	}
	Actual.speed[0] = (u[0][0]+u[0][1]+u[0][2]+u[0][3])/4;
	Actual.speed[1] = (u[1][0]+u[1][1]+u[1][2]+u[1][3])/4;
	Actual.omega = ((u[0][1]-u[0][0]+u[0][3]-u[0][2])/Vehicle.TrackWidth+(u[1][0]-u[1][2]+u[1][1]-u[1][3])/Vehicle.WheelBase)/4;
	Error.speed = Actual.speed - Target.speed;
	Error.omega = Actual.omega - Target.omega;
	//DESIGN FEEDBACKS HERE...
	double r[2][4]
	r[0][0] = -Vehicle.TrackWidth/2;
	r[1][0] =  Vehicle.WheelBase/2;
	r[0][1] =  Vehicle.TrackWidth/2;
	r[1][1] =  Vehicle.WheelBase/2;
	r[0][2] = -Vehicle.TrackWidth/2;
	r[1][2] = -Vehicle.WheelBase/2;
	r[0][3] =  Vehicle.TrackWidth/2;
	r[1][3] = -Vehicle.WheelBase/2;
	
	double Correction[2][4];
	for (int i=0; i<2; i++){
	    for (int j = 0; j<4; j++){
	        Correction[i][j] = -( Error.speed[i]+Error.omega*r[i][j] );
	    }
	}
	double AfterCorrection[2][4];
	AfterCorrection = u + Correction;
	
	#pragma omp parallel for num_threads(4)
	for (int i = 0; i<4; i++){
	    steerVal[i] = atan(AfterCorrection[1][i]/AfterCorrection[0][i]);
	    driveVal[i] = sqrt(AfterCorrection[1][i]*AfterCorrection[1][i]+AfterCorrection[0][i]*AfterCorrection[0][i]);
	}
	
	return 0;
}

Kinematic assessActual()
{
	//rebuild kinematic status
}

int feedback_Call_back()
{
	//control val calc
}


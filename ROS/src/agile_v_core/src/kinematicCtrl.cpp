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

void KinematicCentralizedSteering(double radius, double speed, double* steerVal, double* driveVal)
{
	if (fabs(radius)>=100000)				//filtering out excessively large turns
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

int pidController(Kinematic Target, double* steerActual, double* driveActual, double* steerValCorrection, double* driveValCorrection)
{
	double u[4];
	double v[4];
	Kinematic Error;
	for (int i=0;i<4;i++)
	{
		u[i] = driveActual[i]*cos(steerActual[i]);
		v[i] = driveActual[i]*sin(steerActual[i]);
	}
	Actual.speed[0] = (u(1)+u(2)+u(3)+u(4))/4;
	Actual.speed[1] = (v(1)+v(2)+v(3)+v(4))/4;
	Actual.omega = ((u[2]-u[1]+u[4]-u[3])/Vehicle.TrackWidth+(v[1]-v[3]+v[2]-v[4])/Vehicle.WheelBase)/4;
	Error.speed = Actual.speed - Target.speed;
	Error.omega = Actual.omega - Target.omega;
	//DESIGN FEEDBACKS HERE...
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

int main()
{
	Vehicle = GetVehicleData();
	while (ros::ok())
	{
		//publish control message
	}
}

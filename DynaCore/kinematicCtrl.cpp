//#include <limits.h>

typedef struct VehiclePhysicalParams {
	double TrackWidth;
	double WheelBase;
	double Mass;
	double WheelRadius;
};
typedef struct SensorParams{
	int encoderRes;
	//bla bla bla...
}
VehiclePhysicalParams Vehicle;

/*void GetVehicleData(VehiclePhysicalParams Vehicle)
{
	
}*/

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
		steerVal[1] = atan(Vehicle.WheelBase/2/leftBase);
		steerVal[2] = atan(Vehicle.WheelBase/2/rightBase);
		steerVal[3] = -steerVal[1];
		steerVal[4] = -steerVal[4];
		double omega = speed/radius;
		double Rleft = sqrt(leftBase*leftBase+Vehicle.WheelBase*Vehicle.WheelBase/4);
		double Rright = sqrt(rightBase*rightBase+Vehicle.WheelBase*Vehicle.WheelBase/4);
		driveVal[1] = omega*Rleft;
		driveVal[2] = omega*Rright;
		driveVal[3] = driveVal[1];
		driveVal[4] = driveVal[2];
	}
	//steerVal NEEDS CONVERSION!!!
	//driveVal NEEDS CONVERSION!!!
	return;
}

//#include <limits.h>
#include <float.h>
#include "kinematicCtrl.h"

double step_time;
VehiclePhysicalParams Vehicle;
Kinematic Actual;

void GetVehicleData(int argc, char* argv[])
// 输入车辆物理参数
{
	if (argc==1){
		Vehicle.TrackWidth = 1.315;
		Vehicle.WheelBase = 1.520;
		Vehicle.Mass = 450.0;
		Vehicle.WheelRadius = 0.6114*0.5;
		step_time = 0.04;    //Arduino update time
		//defaults
	}
	else{
		for (int i=0;i<argc;i++){
			if (!strcmp(argv[i],"-TW"))
				Vehicle.TrackWidth = atof(argv[i+1]);
			if (!strcmp(argv[i],"-WB"))
				Vehicle.WheelBase = atof(argv[i+1]);
			if (!strcmp(argv[i],"-M"))
				Vehicle.Mass = atof(argv[i+1]);
			if (!strcmp(argv[i],"-WR"))
				Vehicle.WheelRadius = atof(argv[i+1]);
			if (!strcmp(argv[i],"-ST"))
			    step_time = atof(argv[i+1]);
		}
	}
	return;
}

// 方向盘转向角变成转弯半径
double SteeringWheel2Radius (int SteeringWheelVal, int mode)
{
	switch (mode){
    	case 0:			//Default is using a y=kx+1/x model to cast -32767~32767 to -inf~inf
	    {
	    	return (32767.0/SteeringWheelVal-SteeringWheelVal/32767.0);
	    	break;
	    }
	    case 1:			//Using y=k/[x(+-)b] model
	    {
	    	if (SteeringWheelVal>0)
	    		return (1.5*(32767.0/SteeringWheelVal-1.0));
	    	else if (SteeringWheelVal<0)
	    		return (1.5*(32767.0/SteeringWheelVal+1.0));
	    	else
	    	    return (DBL_MAX);
	    	break;
	    }
    }
}

void KOLCSteering(double radius, double speed, double* steerVal, double* driveVal)
{
    //Kinematic Open-Loop Centralized Steering Controller
	if (fabs(radius)>=100000 && fabs(speed/radius)<=0.0001)
	{   //filtering out excessively slow turns as straight lines
		for (int i=0;i<4;i++){
			steerVal[i] = 0;
			driveVal[i] = speed;
		}
	}
	else
	{
		double leftBase = radius-Vehicle.TrackWidth/2;  //Left Side of Vehicle to Steering Center
		double rightBase = radius+Vehicle.TrackWidth/2; //Right Side ...
		steerVal[0] = atan(Vehicle.WheelBase/2/leftBase);
		steerVal[1] = atan(Vehicle.WheelBase/2/rightBase);
		steerVal[2] = -steerVal[0];
		steerVal[3] = -steerVal[1];
		
		//for (int i = 0; i<4; i++){
    	//    cout << "Steering Angle of Wheel " << i << " :    " << steerVal[i] << endl;
        //}
		
		double omega = speed/radius;
		double Rleft = sqrt(leftBase*leftBase+Vehicle.WheelBase*Vehicle.WheelBase/4); // 左轮的转弯半径
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
	double steerActual[4];
	double driveActual[4];
	for (int i=0; i<4; i++){
        steerActual[i] = Enc[0][i].extractAngle();
	    driveActual[i] = Enc[1][i].extractDiff()*Vehicle.WheelRadius / step_time;
	}
	int errnum = Controller(Target, steerActual, driveActual, steerVal, driveVal);
	//steerVal NEEDS CONVERSION!!!
	//driveVal NEEDS CONVERSION!!!
	return;
}


void KOLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal)
{
    //Kinematic Open-Loop Heading-locked Steering
    steerVal[0] = steering_wheel_input/32767.0*M_PI*0.5;
	steerVal[1] = steerVal[0];
	steerVal[2] = steerVal[0];
	steerVal[3] = steerVal[0];
	driveVal[0] = speed;
	driveVal[1] = speed;
	driveVal[2] = speed;
	driveVal[3] = speed;
    return;
}


void KCLHSteering(int16_t steering_wheel_input, double speed, double* steerVal, double* driveVal)
{
    //Kinematic Closed-Loop Heading-locked Steering
    Kinematic Target;
    Target.speed [1] = speed*sin(steering_wheel_input/32767*M_PI*0.5);
    Target.speed [0] = speed*cos(steering_wheel_input/32767*M_PI*0.5);
    Target.omega = 0;
    double steerActual[4];
	double driveActual[4];
	for (int i=0; i<4; i++){
        steerActual[i] = Enc[0][i].extractAngle();
	    driveActual[i] = Enc[1][i].extractDiff()*Vehicle.WheelRadius / step_time;
	}
    int errnum = Controller(Target, steerActual, driveActual, steerVal, driveVal);
    return;
}


int Controller(Kinematic Target, double* steerActual, double* driveActual, double* steerVal, double* driveVal)
{   //Given Target and Actual, output steer angles and driving linear velocities of wheels
	double u[2][4]; // 第一个是x方向，第二个是y方向，固连车辆坐标系，四个轮子的速度
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
	Error.speed[0] = Actual.speed[0] - Target.speed[0];
	Error.speed[1] = Actual.speed[1] - Target.speed[1];
	Error.omega = Actual.omega - Target.omega;
	//DESIGN FEEDBACKS HERE...
	double r[2][4];
	r[0][0] = -Vehicle.TrackWidth/2;
	r[1][0] =  Vehicle.WheelBase/2;
	r[0][1] =  Vehicle.TrackWidth/2;
	r[1][1] =  Vehicle.WheelBase/2;
	r[0][2] = -Vehicle.TrackWidth/2;
	r[1][2] = -Vehicle.WheelBase/2;
	r[0][3] =  Vehicle.TrackWidth/2;
	r[1][3] = -Vehicle.WheelBase/2;
	
	double Correction[2][4];
	double AfterCorrection[2][4];
	for (int i=0; i<2; i++){
	    #pragma omp parallel for num_threads(4)
	    for (int j = 0; j<4; j++){
	        Correction[i][j] = -( Error.speed[i]+Error.omega*r[i][j] );
	        AfterCorrection[i][j] = u[i][j] + Correction[i][j];
	    }
	}
	
	#pragma omp parallel for num_threads(4)
	for (int i = 0; i<4; i++){
	    steerVal[i] = atan(AfterCorrection[1][i]/AfterCorrection[0][i]);
	    driveVal[i] = sqrt(AfterCorrection[1][i]*AfterCorrection[1][i]+AfterCorrection[0][i]*AfterCorrection[0][i]);
	    Torque[i] = sqrt(Correction[0][i]*Correction[0][i]+Correction[1][i]*Correction[1][i]);
	}
	
	return 0;
}

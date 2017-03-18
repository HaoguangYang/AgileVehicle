#include "ros/ros.h"
#include "kinematicCtrl.h"
#include "subscribers.h"
#include "end_publisher.h"
#include "agile_v_core/joyinfoex.h"

double steerVal[4] = {0};
double driveVal[4] = {0};

void Call_back(const agile_v_core::joyinfoex& controlInput)
{
	int16_t steeringIn = controlInput.dwXpos;
	double speed = controlInput.dwZpos/32767*10;
	double radius = SteeringWheel2Radius(steeringIn, 1);
	
	KCLCSteering(radius, speed, steerVal, driveVal);
	//control val calc
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "dynamic_core");
	ros::NodeHandle handle;
	GetVehicleData(argc, argv);
	ros::Subscriber joystick_input = handle.subscribe("SteeringWheel", 10, Call_back);
	ros::Subscriber WheelActual[4];
	WheelActual[0] = handle.subscribe("WheelActual00", 5, readFromWheelsDrv00);
    WheelActual[1] = handle.subscribe("WheelActual00", 5, readFromWheelsDrv01);
    WheelActual[2] = handle.subscribe("WheelActual00", 5, readFromWheelsDrv02);
	WheelActual[3] = handle.subscribe("WheelActual00", 5, readFromWheelsDrv03);
	
	while (ros::ok){
		publishToWheels(steerVal, driveVal, Torque);
	}
}


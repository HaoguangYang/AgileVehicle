#include <ros.h>
#include "kinematicCtrl.h"
#include "end_publisher.h"
#include "steering_wheel/joyinfoex.h"

void Call_back(const steering_wheel::joyinfoex& controlInput)
{
	int16_t steeringIn = controlInput.dwXpos;
	double speed = controlInput.dwZpos/32767*10;
	double radius = SteeringWheel2Radius(steeringIn, 1);
	KCLCSteering(radius, speed, steerVal, driveVal);
	//control val calc
}

int main(int argc, _TCHAR* argv[])
{
	ros::init(argc, argv, "dynamic_core");
	ros::NodeHandle handle;
	GetVehicleData(argc, argv);
	ros::Subscriber joystick_input = handle.subscribe("WheelActual01", 10, Call_back);
	
	while (ros::ok){
		publishToWheels(steerVal, driveVal, Torque);
	}
}

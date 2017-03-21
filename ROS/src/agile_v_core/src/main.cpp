#include "ros/ros.h"
#include "kinematicCtrl.h"
#include "subscribers.h"
#include "end_publisher.h"
#include "agile_v_core/joyinfoex.h"

double steerVal[4] = {0};
double driveVal[4] = {0};
double Torque[4] = {0};
Encoder Enc[2][4];      //[0][*] - Steering; [1][*] - Driving
bool IsZeroCorrect[4] = {false, false, false, false};

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
    for (int i=1; i<2; i++){
        for (int j=1; j<4; j++){
            Enc[i][j] = Encoder();
        }
    }                   //Encoders Initialize
	ros::init(argc, argv, "dynamic_core");
	ros::NodeHandle handle;
	GetVehicleData(argc, argv);
	
	//Subscribers
	ros::Subscriber WheelActual[4];
	WheelActual[0] = handle.subscribe("WheelActual00", 5, readFromWheelsDrv00);
    WheelActual[1] = handle.subscribe("WheelActual01", 5, readFromWheelsDrv01);
    WheelActual[2] = handle.subscribe("WheelActual02", 5, readFromWheelsDrv02);
	WheelActual[3] = handle.subscribe("WheelActual03", 5, readFromWheelsDrv03);
	ros::Subscriber Electric[4];
	Electric[0] = handle.subscribe("UnitPower00", 5, readFromWheelsPwr00);
	Electric[1] = handle.subscribe("UnitPower01", 5, readFromWheelsPwr01);
	Electric[2] = handle.subscribe("UnitPower02", 5, readFromWheelsPwr02);
	Electric[3] = handle.subscribe("UnitPower03", 5, readFromWheelsPwr03);
	
	//Initiallize the vehicle and start controls.
	bool IsErr = false; //vehicleInit()
	for (int i = 0; i < 4; i++)
	    IsZeroCorrect[i] = !IsErr;
	
	usleep(25000);
	
	ros::Subscriber joystick_input = handle.subscribe("SteeringWheel", 10, Call_back);
	
	while (ros::ok){
		publishToWheels(steerVal, driveVal, Torque);
		publishToUser();
		usleep(25000);
		ros::spinOnce();
	}
}


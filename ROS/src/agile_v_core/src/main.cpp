#include "ros/ros.h"
#include "GUI.h"
#include "kinematicCtrl.h"
#include "subscribers.h"
#include "end_publisher.h"
#include "globals.h"
#include "agile_v_core/joyinfoex.h"

double steerVal[4] = {0};
double driveVal[4] = {0};
double Torque[4] = {0};
Encoder Enc[2][4];      //[0][*] - Steering; [1][*] - Driving
bool IsZeroCorrect[4] = {false, false, false, false};

void Call_back(const agile_v_core::joyinfoex& controlInput)
{
	int16_t steeringIn = controlInput.dwXpos-32768;     //from uint to int
	double speed = (65535.0-controlInput.dwZpos)/32767.0*190.0;
	double radius = SteeringWheel2Radius(steeringIn, 1);
	
	//cout << "Radius: " << radius << endl;
	
	GUIUpdateInput(controlInput);
	
	KOLCSteering(radius, speed, steerVal, driveVal);
	//control val calc
	
	//KOLHSteering(steeringIn, speed, steerVal, driveVal);
}

int main(int argc, char* argv[])
{
    for (int i=0; i<2; i++){
        for (int j=0; j<4; j++){
            Enc[i][j] = Encoder();
        }
    }                   //Encoders Initialize
	ros::init(argc, argv, "dynamic_core");
	ros::NodeHandle handle;
	GetVehicleData(argc, argv);
	GUI_Init();
	
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
	bool IsErr = false; //To be fulfilled by vehicleInit()
	for (int i = 0; i < 4; i++)
	    IsZeroCorrect[i] = !IsErr;
	
	usleep(25000);
	
	ros::Subscriber joystick_input = handle.subscribe("SteeringWheel", 10, Call_back);
	
	//Publishers Initiallize
	//To Wheel
	std_msgs::UInt16MultiArray WheelCtrl[4];
	ros::Publisher wheel_pub[4] = {handle.advertise<std_msgs::UInt16MultiArray>("WheelControl00",2), \
                                   handle.advertise<std_msgs::UInt16MultiArray>("WheelControl01",2), \
                                   handle.advertise<std_msgs::UInt16MultiArray>("WheelControl02",2), \
                                   handle.advertise<std_msgs::UInt16MultiArray>("WheelControl03",2)};
	for (int i=0; i<4; i++){
        WheelCtrl[i].layout.dim.push_back(std_msgs::MultiArrayDimension());
        WheelCtrl[i].layout.dim[0].label = "WheelControl";
        WheelCtrl[i].layout.dim[0].size = 4;
        WheelCtrl[i].layout.dim[0].stride = 1*4;
        //Initialize the Array
        for (int j = 0; j < 4; j++){
            WheelCtrl[i].data.push_back(0);               
        }
    }
    //To UI
    ros::Publisher kineStat = handle.advertise<agile_v_core::kinematics>("VehicleKinematics",5);
    
	
	//ROS Loop
	while (ros::ok()){
		publishToWheels(handle, wheel_pub, WheelCtrl, steerVal, driveVal, Torque);
		publishToUser(handle, kineStat);
		usleep(25000);
		ros::spinOnce();
	}
	SDL_Cleanup();
}


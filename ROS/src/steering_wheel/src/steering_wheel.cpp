#include <stdio.h>
#include "../include/auto_tchar.h"
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <bitset>
#include "ros/ros.h"
#include "steering_wheel/joyinfoex.h"
#include "std_msgs/UInt16MultiArray.h"
//#include "std_msgs/Int32MultiArray.h"

#include <SDL2/SDL.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <errno.h>

using namespace std;

/*typedef struct {
  char end_1;
  char end_2;
  unsigned short Steer;
  unsigned short Drive;
  float Voltage;
  float CurrentS;
  float CurrentD;
} serial_format;*/

steering_wheel::joyinfoex joyinfo;
SDL_Joystick *joy;
bool isOneWheelDebug = true;

// application reads from the specified serial port and reports the collected data

int setup(void)
{
    printf("Welcome to AgileV Steering Wheel Control Utilities!\n\n");

    string PORTNAMEIN;
    cout << "Input Serial Port (e.g. /dev/ttyUSB0): " << endl;
    cin >> PORTNAMEIN;
	string COMMANDCAST = "rosrun rosserial_python serial_node.py ";
    COMMANDCAST = COMMANDCAST+PORTNAMEIN;
	char *COMMAND = new char [COMMANDCAST.length() + 1];
    std::strcpy(COMMAND, COMMANDCAST.c_str());
    system(COMMAND);			//Open Arduino port for ROS interface.
	
	unsigned int JOYSTICKID1;
	if (SDL_Init(SDL_INIT_JOYSTICK) < 0){
        cout << "Error initializing SDL!" << endl;
        return -1;
    }
    
    int CtrlNum = SDL_NumJoysticks();
    if (CtrlNum == 1)
        JOYSTICKID1 = 0;
    else{
        cout << "There are " << CtrlNum << " controllers found..." << endl;
        for(int i=0;i<CtrlNum;i++)
        {
            joy = SDL_JoystickOpen(i);
            printf("%s\n", SDL_JoystickName(joy));
        }
        cout << "Choose the one you wish to use: " << endl;
        cin >> JOYSTICKID1;
    }
    return JOYSTICKID1;
}


int FFupdate(SDL_Joystick * joystick , unsigned short center) 
{
 SDL_Haptic *haptic;
 SDL_HapticEffect effect;
 int effect_id;

 // Open the device
 haptic = SDL_HapticOpenFromJoystick( joystick );
 if (haptic == NULL) return -1; // Most likely joystick isn't haptic

 // See if it can do sine waves
 if ((SDL_HapticQuery(haptic) & SDL_HAPTIC_SPRING)==0) {
  SDL_HapticClose(haptic); // No sine effect
  return -1;
 }

 // Create the effect
 memset( &effect, 0, sizeof(SDL_HapticEffect) ); // 0 is safe default
 effect.type = SDL_HAPTIC_SPRING;
 effect.condition.length = 30; // 30ms long
 effect.condition.delay = 0; // no delay
 effect.condition.center[0] = (int16_t)(center-32768);//NEED CONVERSION!!!

 // Upload the effect
 effect_id = SDL_HapticNewEffect( haptic, &effect );

 // Test the effect
 SDL_HapticRunEffect( haptic, effect_id, 1 );
 SDL_Delay( 5000); // Wait for the effect to finish

 // We destroy the effect, although closing the device also does this
 SDL_HapticDestroyEffect( haptic, effect_id );

 // Close the device
 SDL_HapticClose(haptic);

 return 0; // Success
}


void ActuaterFeedback(const std_msgs::UInt16MultiArray& ActuatorStatus)
{
	if (isOneWheelDebug)
	{
		int errNum = FFupdate(joy,ActuatorStatus.data[0]);
		cout << "ActualSteer: " << ActuatorStatus.data[0];
		cout << "ActualDrive: " << ActuatorStatus.data[1];
	}
	return;
}


int main(int argc, _TCHAR* argv[])
{
// connect the COM
	ros::init(argc, argv, "steering_whel");
	ros::NodeHandle handle;
	ros::Publisher steering_wheel_pub = handle.advertise<steering_wheel::joyinfoex>("SteeringWheel",10);
	
	int JOYSTICKID1 = setup();
	
	//if (isOneWheelDebug)
	ros::Subscriber actuator_feedback = handle.subscribe("WheelActual-1", 10, ActuaterFeedback);
	ros::Publisher wheel_pub = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl-1",10);
	std_msgs::UInt16MultiArray WheelCtrl;
	
	malloc(sizeof(std_msgs::MultiArrayDimension) * 4);
	WheelCtrl.layout.dim[0].label = "UnitPower";
    WheelCtrl.layout.dim[0].size = 4;
    WheelCtrl.layout.dim[0].stride = 1*4;
    
	uint16_t driveDutycycle=0;
	uint16_t breaking = 255;
	uint16_t steer=2048;
    const int encoder_resolution=4096;
    const int fullDutycycle=255;	
	
    //Serial* SP = new Serial(PORTNAME);
	//if (SP->IsConnected())
	//	printf("We're connected!\n");

//data communication
	/*char incomingData[256] = "";			// don't forget to pre-
	int dataLengthin = 127;
	int readResult = 0;
	char outSteer[6]="2048s";
	int driveDutycycle=0;
	char outDrive[5]="001d";
	char outBreak[5]="100b"*/

	

//joystick initialize***********************

    joy=SDL_JoystickOpen(JOYSTICKID1);
    if (joy) {
        printf("Opened Joystick %d\n",JOYSTICKID1);
        printf("Name: %s\n", SDL_JoystickNameForIndex(0));
        printf("Number of Axes: %d\n", SDL_JoystickNumAxes(joy));
        printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(joy));
        printf("Number of Balls: %d\n", SDL_JoystickNumBalls(joy));
    } else {
        printf("Couldn't open Joystick %d\n", JOYSTICKID1);
        return -1*JOYSTICKID1;
    }

//button status
	bool is_in_situ=false;
	bool is_any_direction=false;
	bool is_tradition=true;
	bool now_in_situ=false;
	bool now_any_direction=false;
	bool now_tradition=false;
	bool last_in_situ=false;
	bool last_any_direction=false;
	bool last_tradition=false;

	//int count = 0;
//start control
	bool action = 1;
    SDL_Event SysEvent;
    bool NoQuit = true;
    while(NoQuit && ros::ok())
    {
    	if (SDL_PollEvent(&SysEvent))
        {
            switch (SysEvent.type)
            {
                case SDL_JOYAXISMOTION:
                {
                    joyinfo.dwXpos = SDL_JoystickGetAxis(joy,0) + 32767 + 1 ;
	    	        joyinfo.dwYpos = SDL_JoystickGetAxis(joy,1) + 32767 + 1 ;
	    	        joyinfo.dwZpos = SDL_JoystickGetAxis(joy,2) + 32767 + 1 ;
	    	        joyinfo.dwRpos = SDL_JoystickGetAxis(joy,3) + 32767 + 1 ;
	    	        //Only 4 axis.
		            //joyinfo.dwUpos = SDL_JoystickGetAxis(joy,4);
		            //joyinfo.dwVpos = SDL_JoystickGetAxis(joy,5);
	    	        break;
	    	    }
	    	    case SDL_QUIT:
	    	    {
	    	        NoQuit = false;
	    	        break;
	    	    }
	    	}
            SDL_FlushEvents(SDL_APP_TERMINATING, SDL_LASTEVENT); //Flush Old Events
			if(joyinfo.dwZpos != 32767)
				action = 0;
			/*if(action)
			{
				cout << "Z Throttle:" << 65536 << endl; //The original value of the inactivated throttle is 32767, should be modified to 65535 so that throttle is 0		
				cout << "R Break:" << 0.9*65536 << endl;
			}
			else
			{
			cout << "Z Throttle:" << joyinfo.dwZpos << endl; //If activated (moved), output real value.
			cout << "R Break:" << joyinfo.dwRpos << endl; //The same for breaks.
			}
			cout << "buttonNumber" << joyinfo.dwButtonNumber << endl;*/
			cout << "buttonStatus" << bitset<64>(joyinfo.dwButtons) << endl; //Output button status
			
			if (isOneWheelDebug)
			{
			    //preprocessing data
				if(action)//Double-check that the inactivated throttle value is set to 0.
			    {
				    driveDutycycle = 0;
				    breaking = 225;
			    }
			    else
			    {
				    breaking = (uint16_t)((1-joyinfo.dwRpos/65535.0)*fullDutycycle);			//Modify as necessary.
				    if (breaking==0)
					    driveDutycycle = (uint16_t)((1-joyinfo.dwZpos/65535.0)*fullDutycycle);
				    else
					    driveDutycycle = 0;
			    }
			    steer=(uint16_t)(joyinfo.dwXpos/65535.0*encoder_resolution);
			    //Output monitoring
			    cout << "X Steering Wheel:" << steer << endl;
			    cout << "Z Throttle:" << driveDutycycle << endl;
				cout << "R Break:" << breaking << endl;
				
				//Verifying data and publishing
			    if(driveDutycycle>=0 && driveDutycycle<=255 && breaking>=0 && breaking<=255 && steer>=0 && steer<=encoder_resolution)
			    {
			        /***REFER TO ARDUINO PROGRAM***
                    std_msgs::UInt16MultiArray ctrl_var;
                    uint16 inputSteer;
                    uint16 inputDrive;
                    uint16 inputBreak;
                    uint16 reverse;
                    ************/
				    WheelCtrl.data[0] = steer;
				    WheelCtrl.data[1] = driveDutycycle;
				    WheelCtrl.data[2] = breaking;
				    WheelCtrl.data[3] = 0;
			    }
			    wheel_pub.publish(WheelCtrl);
			//Publish steering wheel data to one wheel for debugging
			}
			/*printf("outDrive:%s\n",outDrive);
			bool isWriteDrive = SP->WriteData((char*)&outDrive, strlen(outDrive));
			cout << "WriteSucceed?" << isWriteDrive << endl;
			printf("outSteer:%s\n",outSteer);
			bool isWriteSteer = SP->WriteData((char*)&outSteer, strlen(outSteer));
			cout << "WriteSucceed?" << isWriteSteer << endl;
			printf("outBreak:%s\n",outBreak);
			bool isWriteBreak = SP->WriteData((char*)&outBreak, strlen(outBreak));
			cout << "WriteSucceed?" << isWriteBreak << endl;*/
		}       //If have sysevent then update joystick values
		//usleep(120000);
		system("clear");
		ros::spin();
		//readResult = SP->ReadData(incomingData,dataLengthin);
		///*if(readResult<2*sizeof(serial_format)){
		//	printf("No enough data received. Let's wait.\n");
		//	Sleep(1500);continue;
		//}*/

		/*int ptr=0;
		while(!(incomingData[ptr]=='?' && incomingData[ptr+1]=='?') && ptr<readResult-sizeof(serial_format)) {
			ptr++;
		}
		serial_format* ptr_to_first_valid = (serial_format*) &incomingData[ptr];

		printf("Actual Steer: %d\n",ptr_to_first_valid->Steer);
		printf("Actual Drive: %d\n",ptr_to_first_valid->Drive);
		printf("Bus Voltage: %f\n", ptr_to_first_valid->Voltage);
		printf("Current on Driving: %f\n", ptr_to_first_valid->CurrentD);
		printf("Current on Steering: %f\n", ptr_to_first_valid->CurrentS);
		printf("Power Consumed on This Unit: %f\n", (ptr_to_first_valid->Voltage)*((ptr_to_first_valid->CurrentD)+(ptr_to_first_valid->CurrentS)));*/
        //However always read serial data.

		
    }
    //SP->~Serial();

	SDL_JoystickClose(joy);
    
	system("pause");
	return 0;
}

#include "steering_wheel.h"

using namespace std;

steering_wheel::joyinfoex joyinfo;
SDL_Joystick *joy;
bool isOneWheelDebug = true;

// application reads from the specified serial port and reports the collected data

int setup(void)
{
    system("clear");
	printf("Welcome to AgileV Steering Wheel Control Utilities!\n\n");
	
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

// 力反馈，现在还不能用
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
    SDL_HapticClose(haptic); // No spring effect
    return -2;
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

// 单轮驱动用来显示方向盘示数
void ActuaterFeedback(const std_msgs::UInt16MultiArray& ActuatorStatus)
{
    system("clear");
	if (isOneWheelDebug)
	{
	    int errNum = FFupdate(joy,ActuatorStatus.data[0]);
	    cout << "ActualSteer: " << ActuatorStatus.data[0] << endl;
	    cout << "ActualDrive: " << ActuatorStatus.data[1] << endl;
	    cout << "HapticsUpdateStatus: " << errNum << endl;
	    cout << "Joystick: " << joyinfo.dwXpos << endl;
	    cout << "          " << joyinfo.dwZpos << endl;
	    cout << "Buffer:" <<endl;
	}
	return;
}


int main(int argc, char* argv[])
{
// connect the COM
    ros::init(argc, argv, "steering_wheel");

    ros::NodeHandle handle;

    ros::Publisher steering_wheel_pub = handle.advertise<steering_wheel::joyinfoex>("SteeringWheel",10);
	
    int JOYSTICKID1 = setup();
	
    //if (isOneWheelDebug)
    ros::Subscriber actuator_feedback = handle.subscribe("WheelActual01", 10, ActuaterFeedback);
    ros::Publisher wheel_pub = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl01",2);
    
    // Arduino 接口用
    std_msgs::UInt16MultiArray WheelCtrl;
    
    // ROS Multiarray initialize operation	
    WheelCtrl.layout.dim.push_back(std_msgs::MultiArrayDimension());
    WheelCtrl.layout.dim[0].label = "WheelControl";
    WheelCtrl.layout.dim[0].size = 4;
    WheelCtrl.layout.dim[0].stride = 1*4;

    //Initialize the Array
    for (int i = 0; i < 4; i++){
        WheelCtrl.data.push_back(0);               
    }
	
    uint16_t driveDutycycle=0;
    uint16_t brake = 255;
    uint16_t steer=2048;
    const int encoder_resolution=4095;
    const int fullDutycycle=255;

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

/*	bool is_in_situ=false;
	bool is_any_direction=false;
	bool is_tradition=true;
	bool now_in_situ=false;
	bool now_any_direction=false;
	bool now_tradition=false;
	bool last_in_situ=false;
	bool last_any_direction=false;
	bool last_tradition=false;
*/

	//int count = 0;

    //start control
    bool noAction = 1;
    SDL_Event SysEvent;
    bool NoQuit = true;
    while(NoQuit && ros::ok())
    {
    	if (SDL_PollEvent(&SysEvent)) // 如果游戏杆动了
        {
            switch (SysEvent.type)
            {
                case SDL_JOYAXISMOTION:
                {
                    joyinfo.dwXpos = SDL_JoystickGetAxis(joy,0) + 32768 ; // 转向
	    	    	joyinfo.dwYpos = SDL_JoystickGetAxis(joy,1) + 32768 ; // 离合
	    	    	joyinfo.dwZpos = SDL_JoystickGetAxis(joy,2) + 32768 ; // 油门
	    	    	joyinfo.dwRpos = SDL_JoystickGetAxis(joy,3) + 32768 ;
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
			if(joyinfo.dwZpos != 32767) // 为了防止最开始的时候的跳变
				noAction = 0;
			if(noAction)//Double-check that the inactivated throttle value is set to 0.
			{
			    joyinfo.dwRpos = 0;				//Full Brake
			    joyinfo.dwZpos = 65535;			//Zero Power
			}
			// 发出方向盘信息
			steering_wheel_pub.publish(joyinfo);
			
			if (isOneWheelDebug)
			{
			    //preprocessing data
			    brake = (uint16_t)((1-joyinfo.dwRpos/65535.0)*fullDutycycle);			//Modify as necessary.

			    if (brake==0) 
				    driveDutycycle = (uint16_t)((1-joyinfo.dwZpos/65535.0)*fullDutycycle);
			    else // 刹车的话油门至0
				    driveDutycycle = 0;

				// 转向角度
			    steer=(uint16_t)(encoder_resolution-(joyinfo.dwXpos/65535.0*encoder_resolution));

			    // Output monitoring
				cout << "buttonStatus" << bitset<64>(joyinfo.dwButtons) << endl; //Output button status
			    cout << "X Steering Wheel:" << steer << endl;
			    cout << "Z Throttle:" << driveDutycycle << endl;
				cout << "R Brake:" << brake << endl;
				
				// Verifying data and publishing
			    if(driveDutycycle>=0 && driveDutycycle<=255 && brake>=0 && brake<=255 && steer>=0 && steer<=encoder_resolution) //Double check
			    {
			    /***REFER TO ARDUINO PROGRAM***
                    std_msgs::UInt16MultiArray ctrl_var;
                    uint16 inputSteer;
                    uint16 inputDrive;
                    uint16 inputBrake;
                    uint16 reverse;
                    ************/
				    WheelCtrl.data[0] = steer;
				    WheelCtrl.data[1] = driveDutycycle;
				    WheelCtrl.data[2] = brake;
				    WheelCtrl.data[3] = 0;
			    }
			    
			    wheel_pub.publish(WheelCtrl);
			    usleep(25000);
			//Publish steering wheel data to one wheel for debugging
			}
		}       //If have sysevent then update joystick values
		
		ros::spinOnce();

    }

	SDL_JoystickClose(joy);
	//system("pause");
	return 0;
}

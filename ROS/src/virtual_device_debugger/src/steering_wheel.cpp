#include "steering_wheel.h"
#include <math.h>

using namespace std;

steering_wheel::joyinfoex joyinfo;
bool isOneWheelDebug = false;

// application reads from the specified serial port and reports the collected data

int setup(void)
{
    system("clear");
	printf("Welcome to AgileV Debug Utilities!\n\n");
	
	unsigned int JOYSTICKID1;
	if (SDL_Init(SDL_INIT_JOYSTICK) < 0){
        cout << "Error initializing SDL!" << endl;
        return -1;
    }
    
    int CtrlNum = SDL_NumJoysticks();
    if (CtrlNum == 1)
        JOYSTICKID1 = 0;
    else if (CtrlNum == 0) {
		cout << "There is 0 physical controllers found..." << endl;
		return -2;
	}
	else {
        cout << "There are " << CtrlNum << " physical controllers found..." << endl;
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

double SteeringWheel2Radius (int SteeringWheelVal, int mode)
{
    double value;
	switch (mode){
    	case 0:			//Default is using a y=kx+1/x model to cast -32767~32767 to -inf~inf
	    {
	    	value = 1.0/SteeringWheelVal-SteeringWheelVal/32767.0;
	    	break;
	    }
	    case 1:			//Using y=k/[x(+-)b] model
	    {
	    	if (SteeringWheelVal>=0)
	    		value = 1.0/SteeringWheelVal-1/32767.0;
	    	else
	    		value = 1.0/SteeringWheelVal+1/32767.0;
	    	break;
	    }
    }
    return value;
}

int main(int argc, char* argv[])
{
// connect the COM
    ros::init(argc, argv, "steering_wheel");

    ros::NodeHandle handle;

    ros::Publisher steering_wheel_pub = handle.advertise<steering_wheel::joyinfoex>("SteeringWheel",10);
	
    int JOYSTICKID1 = setup();
	
    //if (isOneWheelDebug)
	//Flip Publisher and Subscriber.
    ros::Publisher actuator_feedback0 = handle.subscribe("WheelActual00", 10, ActuaterFeedback);
    ros::Publisher actuator_feedback1 = handle.subscribe("WheelActual01", 10, ActuaterFeedback);
    ros::Publisher actuator_feedback2 = handle.subscribe("WheelActual02", 10, ActuaterFeedback);
    ros::Publisher actuator_feedback3 = handle.subscribe("WheelActual03", 10, ActuaterFeedback);
    ros::Subscriber wheel_pub0 = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl00",2);
    ros::Subscriber wheel_pub1 = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl01",2);
    ros::Subscriber wheel_pub2 = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl02",2);
    ros::Subscriber wheel_pub3 = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl03",2);
    
    // Arduino 接口用
    std_msgs::UInt16MultiArray WheelCtrl00;
    std_msgs::UInt16MultiArray WheelCtrl01;
    std_msgs::UInt16MultiArray WheelCtrl02;
    std_msgs::UInt16MultiArray WheelCtrl03;
    
    // ROS Multiarray initialize operation	
    WheelCtrl00.layout.dim.push_back(std_msgs::MultiArrayDimension());
    WheelCtrl00.layout.dim[0].label = "WheelControl";
    WheelCtrl00.layout.dim[0].size = 4;
    WheelCtrl00.layout.dim[0].stride = 1*4;
    
    WheelCtrl01.layout.dim.push_back(std_msgs::MultiArrayDimension());
    WheelCtrl01.layout.dim[0].label = "WheelControl";
    WheelCtrl01.layout.dim[0].size = 4;
    WheelCtrl01.layout.dim[0].stride = 1*4;
    
    WheelCtrl02.layout.dim.push_back(std_msgs::MultiArrayDimension());
    WheelCtrl02.layout.dim[0].label = "WheelControl";
    WheelCtrl02.layout.dim[0].size = 4;
    WheelCtrl02.layout.dim[0].stride = 1*4;
    
    WheelCtrl03.layout.dim.push_back(std_msgs::MultiArrayDimension());
    WheelCtrl03.layout.dim[0].label = "WheelControl";
    WheelCtrl03.layout.dim[0].size = 4;
    WheelCtrl03.layout.dim[0].stride = 1*4;

    //Initialize the Array
    for (int i = 0; i < 4; i++){
        WheelCtrl00.data.push_back(0);
        WheelCtrl01.data.push_back(0);
        WheelCtrl02.data.push_back(0);
        WheelCtrl03.data.push_back(0);
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
			    steer=(uint16_t)(joyinfo.dwXpos/65535.0*encoder_resolution);

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
				    WheelCtrl00.data[0] = steer;
				    WheelCtrl00.data[1] = driveDutycycle;
				    WheelCtrl00.data[2] = min((int)(1.5*brake),255);
				    WheelCtrl00.data[3] = 0;
			    }
			    
			    /*/TEST CODE STARTS..............................................
			    double TrackWidth = 1.315;
		        double WheelBase = 1.520;
		        double Mass = 450.0;
		        double WheelRadius = 0.6114*0.5;
			    
			    double radius = SteeringWheel2Radius(joyinfo.dwXpos-32768,0);
			    
			    cout << radius << endl;
			    
			    double leftBase = radius-TrackWidth/2;  //Left Side of Vehicle to Steering Center
		        double rightBase = radius+TrackWidth/2; //Right Side ...
		        if (leftBase == 0) leftBase = 0.00001;
		        if (rightBase == 0) rightBase = 0.00001;
		        uint16_t steerVal[4];
		        steerVal[0] = 4095-((uint16_t)(atan(WheelBase/2/leftBase)*(encoder_resolution+1)*0.5/M_PI)+encoder_resolution+1)%(encoder_resolution+1);
		        steerVal[1] = 4095-((uint16_t)(atan(WheelBase/2/rightBase)*(encoder_resolution+1)*0.5/M_PI)+encoder_resolution+1)%(encoder_resolution+1);
		        steerVal[2] = steerVal[0];
		        steerVal[3] = steerVal[1];
		        
		        WheelCtrl00.data[0] = steerVal[0];    //for test code*/
			    wheel_pub0.publish(WheelCtrl00);
			    
			    //WheelCtrl01.data[0] = steerVal[1];    //for test code
			    wheel_pub1.publish(WheelCtrl00);
			    
			    //WheelCtrl02.data[0] = steerVal[2];    //for test code
			    wheel_pub2.publish(WheelCtrl00);
			    
			    //WheelCtrl03.data[0] = steerVal[3];    //for test code
			    wheel_pub3.publish(WheelCtrl00);
			    
			    //TEST CODE ENDS................................................
			    
			    usleep(25000);
			//Publish steering wheel data to one wheel for debugging
			}
		}       //If have sysevent then update joystick values
		
		
		ros::spinOnce();

    }
    
    WheelCtrl00.data[1] = 0;//driveDutycycle;
	WheelCtrl00.data[2] = 255;
	wheel_pub0.publish(WheelCtrl00);
	wheel_pub1.publish(WheelCtrl00);
	wheel_pub2.publish(WheelCtrl00);
	wheel_pub3.publish(WheelCtrl00);
    ros::spinOnce();
    
	SDL_JoystickClose(joy);
	//system("pause");
	return 0;
}

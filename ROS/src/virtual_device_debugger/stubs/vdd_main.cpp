#include "steering_wheel.h"
#include "encoder.h"
#include <math.h>

//This code is now a stub to be integrated with agile_v_core, migrating the GUI part, and now the control part.

using namespace std;

steering_wheel::joyinfoex joyinfo;
SDL_Joystick *joy;


// application reads from the specified serial port and reports the collected data

int setup(void)
{
    system("clear");
	printf("Welcome to AgileV Debug Utilities!\n\n");
	
	unsigned int JOYSTICKID1;
	if (SDL_Init(SDL_INIT_JOYSTICK) < 0){
        fprintf( stderr, "Could not initialise SDL: %s\n", SDL_GetError() );
        exit( -1 );
    }
    
    
    
    int CtrlNum = SDL_NumJoysticks();
    if (CtrlNum == 1)
        JOYSTICKID1 = 0;
    else{
        if (CtrlNum == 0) {
		    cout << "There is 0 physical controllers found..." << endl;
		    JOYSTICKID1 = -2;
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

// 单轮驱动用来显示控制数据
void SteerFeedback(const std_msgs::UInt16MultiArray& FFStatus)
{
    system("clear");
    int errNum = FFupdate(joy,FFStatus.data[0]);
	cout << "Actual Drive Direction  : " << FFStatus.data[0] << endl;
	cout << "Actual Heading Direction: " << FFStatus.data[1] << endl;
	cout << "Haptics Update Status   : " << errNum << endl;
	cout << "Joystick Status         : " << joyinfo.dwXpos << endl;
	cout << "                          " << joyinfo.dwZpos << endl;
	cout << "Buffer:" <<endl;
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
	
    int JOYSTICKID1 = setup();
    
    ros::Publisher steering_wheel_pub = handle.advertise<steering_wheel::joyinfoex>("SteeringWheel",10);
	ros::Subscriber steer_feedback = handle.subscribe("SteeringWheelFeedBack", 10, SteerFeedback);
	

//joystick initialize***********************

	if (JOYSTICKID1>=0){
		joy=SDL_JoystickOpen(JOYSTICKID1);
		if (joy) {
			printf("Opened Joystick %d\n",JOYSTICKID1);
			printf("Name: %s\n", SDL_JoystickNameForIndex(0));
			printf("Number of Axes: %d\n", SDL_JoystickNumAxes(joy));
			printf("Number of Buttons: %d\n", SDL_JoystickNumButtons(joy));
			printf("Number of Balls: %d\n", SDL_JoystickNumBalls(joy));
		}
		else {
			printf("Couldn't open Joystick %d\n", JOYSTICKID1);
			printf("Using Keyboard Input as Virtual Control Device...");
		}
	}
	else {
		cout << "Using Keyboard Input as Virtual Control Device..." << endl;
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
	
	if (JOYSTICKID1>=0){
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
	    	    	joyinfo.dwRpos = SDL_JoystickGetAxis(joy,3) + 32768 ; // 刹车
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
			
			//GUIUpdateInput();
			    
			usleep(25000);
			//Publish steering wheel data to one wheel for debugging
		}       //If have sysevent then update joystick values
		ros::spinOnce();
    }
	SDL_JoystickClose(joy);
	}
	else{
		joyinfo.dwXpos = 32768;
		joyinfo.dwRpos = 65535;			//Zero Brake
		joyinfo.dwZpos = 65535;			//Zero Power
		
		while(NoQuit && ros::ok())
		{
		    //cin >> joyinfo.dwXpos;
		    //cin >> joyinfo.dwRpos;
		    //cin >> joyinfo.dwZpos;
			int delta_x, delta_z;
			if (SDL_PollEvent(&SysEvent))
			{
				switch (SysEvent.type)
				{
					case SDL_KEYDOWN:
					// Check the SDLKey values and move change the coords
						switch( SysEvent.key.keysym.sym ){
						case SDLK_LEFT:
							delta_x = max(delta_x-1, -255);
							break;
						case SDLK_RIGHT:
							delta_x = min(delta_x+1, 255);
							break;
						case SDLK_UP:
							delta_z = max(delta_z-1, -255);
							break;
						case SDLK_DOWN:
							delta_z = min(delta_z+1, 255);
							break;
						default:
							break;
						}
						break;
					// We must also use the SDL_KEYUP events to zero the x 
					// and y velocity variables. But we must also be       
					// careful not to zero the velocities when we shouldn't
					case SDL_KEYUP:
						switch( SysEvent.key.keysym.sym ){
						case SDLK_LEFT:
                        // We check to make sure the alien is moving 
                        // to the left. If it is then we zero the    
                        // velocity. If the alien is moving to the   
                        // right then the right key is still press   
                        // so we don't tocuh the velocity            
							if( delta_x < 0 )
								delta_x = 0;
							break;
						case SDLK_RIGHT:
							if( delta_x > 0 )
								delta_x = 0;
							break;
						case SDLK_UP:
							if( delta_z < 0 )
								delta_z = 0;
							break;
						case SDLK_DOWN:
							if( delta_z > 0 )
								delta_z = 0;
							break;
						default:
							break;
						}
						break;
					case SDL_QUIT:
	    	        {
	    	            NoQuit = false;
	    	            break;
	    	        }
					default:
						break;
				}
			}
			//SDL_FlushEvents(SDL_APP_TERMINATING, SDL_LASTEVENT); //Flush Old Events
			//cout << "Done" << endl;
			if (delta_x != 0 | delta_z != 0){
				joyinfo.dwXpos = max(min(int(joyinfo.dwXpos+delta_x),65535),0);
				if (delta_z < 0){
					if (joyinfo.dwRpos==65535)
						joyinfo.dwZpos = max(min(int(joyinfo.dwZpos)+delta_z,65535),0);
					else
						joyinfo.dwRpos = max(min(int(joyinfo.dwRpos)-delta_z,65535),0);
				}
				if (delta_z > 0){
					if (joyinfo.dwZpos==65535)
						joyinfo.dwRpos = max(min(int(joyinfo.dwRpos)-delta_z,65535),0);
					else
						joyinfo.dwZpos = max(min(int(joyinfo.dwZpos)+delta_z,65535),0);
				}
				steering_wheel_pub.publish(joyinfo);
				
				//GUIUpdateInput();
	/****/
	system("clear");
	cout << "Joystick Status         : " << joyinfo.dwXpos << endl;
	cout << "                          " << joyinfo.dwRpos << endl;
	cout << "                          " << joyinfo.dwZpos << endl;
	/****/
				
			}
			usleep(25000);
			ros::spinOnce();
		}
	}
	
	//system("pause");
	return 0;
}


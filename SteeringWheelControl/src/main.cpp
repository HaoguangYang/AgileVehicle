#include <stdio.h>
#include "auto_tchar.h"
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <bitset>
#include "SerialClass.h"	// Library described above

#if defined(_MSC_VER)
#include <windows.h>
#include <mmsystem.h>
#pragma comment(lib,"winmm.lib")

#elif defined(__linux__)
#include <SDL2/SDL.h>
#include <sys/time.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/joystick.h>
#include <errno.h>

#endif
using namespace std;

typedef struct {
  char end_1;
  char end_2;
  unsigned short x;
  unsigned short y;
} serial_format;

#ifdef __linux__
  typedef struct joyinfoex_tag {
    unsigned int dwXpos;
    unsigned int dwYpos;
    unsigned int dwZpos;
    unsigned int dwRpos;
    unsigned int dwUpos;
    unsigned int dwVpos;
    unsigned long dwButtons;
    unsigned int dwButtonNumber;
    unsigned long dwPOV;
    } JOYINFOEX;
#endif

// application reads from the specified serial port and reports the collected data
int _tmain(int argc, _TCHAR* argv[])
{
// connect the COM
	printf("Welcome to the serial test app!\n\n");
#if defined (_MSC_VER)
	Serial* SP = new Serial("\\\\.\\COM6");    // adjust as needed
#elif defined (__linux__)
    string PORTNAMEIN;
    cout << "Input Serial Port (e.g. /dev/ttyACM0): " << endl;
    cin >> PORTNAMEIN;
    char *PORTNAME = new char [PORTNAMEIN.length() + 1];
    std::strcpy(PORTNAME, PORTNAMEIN.c_str());
	Serial* SP = new Serial(PORTNAME);
	unsigned int JOYSTICKID1;
#endif

	if (SP->IsConnected())
		printf("We're connected!\n");

//data communication
	char incomingData[256] = "";			// don't forget to pre-
	int dataLengthin = 127;
	int readResult = 0;

	int steer=2048;
	const int encoder_resolution=4096;
	char outSteer[6]="2048s";
	int driveDutycycle=0;
	const int fullDutycycle=255;
	char outDrive[5]="001d";
	char outBreak[5]="100b"

//joystick initialize***********************

	joyinfoex_tag joyinfo;

#ifdef __linux__
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0){
        cout << "Error initializing SDL!" << endl;
        return 1;
    }
	SDL_Joystick *joy;
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
#endif

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
#if defined(_MSC_VER)
	while(SP->IsConnected())                            // CYCLES HERE...
	{
		//acquire joystick info **********************
		joyGetPosEx(JOYSTICKID1, &joyinfo);
#elif defined(__linux__)
    SDL_Event SysEvent;
    bool NoQuit = true;
    while(SP->IsConnected() && NoQuit)
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
#endif
		cout << "X Steering Wheel:" << joyinfo.dwXpos << endl;
		if(joyinfo.dwZpos != 32767)
			action = 0;
		if(action)
		{
			cout << "Z Throttle:" << 65536 << endl; //The original value of the inactivated throttle is 32767, should be modified to 65535 so that throttle is 0		
			cout << "R Break:" << 0.9*65536 << endl;
		}
		else
		{
			cout << "Z Throttle:" << joyinfo.dwZpos << endl; //If activated (moved), output real value.
			cout << "R Break:" << joyinfo.dwRpos << endl; //The same for breaks.
		}
		cout << "buttonNumber" << joyinfo.dwButtonNumber << endl;
		cout << "buttonStatus" << bitset<64>(joyinfo.dwButtons) << endl; //Output button status
		//Buttons:
		/*now_in_situ=bitset<64>(joyinfo.dwButtons)[0];
		now_any_direction=bitset<64>(joyinfo.dwButtons)[3];
		now_tradition=bitset<64>(joyinfo.dwButtons)[1];
		cout << "is_in_situ:" << is_in_situ << endl;
		cout << "is_any_direction:" << is_any_direction << endl;
		cout << "is_tradition:" << is_tradition << endl;
		
		if(now_in_situ== true && last_in_situ ==false)
		{
			is_in_situ =true;
			is_any_direction =false;
			is_tradition =false;

			SP->WriteData("b",1);
		}

		if(now_any_direction== true && last_any_direction ==false)
		{
			is_in_situ =false;
			is_any_direction =true;
			is_tradition =false;

			SP->WriteData("f",1);

		}

		if(now_tradition== true && last_tradition ==false)
		{
			is_in_situ =false;
			is_any_direction =false;
			is_tradition =true;
			SP->WriteData("f",1);

		}

		last_in_situ =now_in_situ;
		last_any_direction =now_any_direction;
		last_tradition =now_tradition;*/
		
		if(action)//Double-check that the inactivated throttle value is set to 0.
		{
			driveDutycycle = 0;
			breaking = 225;
		}
		else
		{
			breaking = (int)((1-joyinfo.dwRpos/65535.0)*fullDutycycle);			//Modify as necessary.
			if (braking=0)
				driveDutycycle = (int)((1-joyinfo.dwZpos/65535.0)*fullDutycycle);
			else
				driveDutycycle = 0;
		}
		if(driveDutycycle>=0 && driveDutycycle<=255)
		{
			for (int k=0;k<3;k++)
			{
				outDrive[2-k]=char(driveDutycycle%10 + '0');
				driveDutycycle=driveDutycycle/10;
			}
		}
		if(breaking>=0 && breaking<=255)
		{
			for (int k=0;k<3;k++)
			{
				outBreak[2-k]=char(breaking%10 + '0');
				breaking=breaking/10;
			}
		}
		steer=(int)(joyinfo.dwXpos/65535.0*encoder_resolution);
		if(steer>=0 && steer<=encoder_resolution)
		{
			for (int k=0;k<4;k++)
			{
				outSteer[3-k]=char(steer%10 + '0');
				steer=steer/10;
			}
		}
		
		printf("outDrive:%s\n",outDrive);
		bool isWriteDrive = SP->WriteData((char*)&outDrive, strlen(outDrive));
		cout << "WriteSucceed?" << isWriteDrive << endl;
		printf("outSteer:%s\n",outSteer);
		bool isWriteSteer = SP->WriteData((char*)&outSteer, strlen(outSteer));
		cout << "WriteSucceed?" << isWriteSteer << endl;
		printf("outBreak:%s\n",outBreak);
		bool isWriteBreak = SP->WriteData((char*)&outBreak, strlen(outBreak));
		cout << "WriteSucceed?" << isWriteBreak << endl;
#if defined(_MSC_VER)
		Sleep(120);
		system("cls");
#elif defined (__linux__)
		}       //If have sysevent then update joystick values
		usleep(120000);
		system("clear");
#endif
		readResult = SP->ReadData(incomingData,dataLengthin);
		///*if(readResult<2*sizeof(serial_format)){
		//	printf("No enough data received. Let's wait.\n");
		//	Sleep(1500);continue;
		//}*/

		int ptr=0;
		while(!(incomingData[ptr]=='?' && incomingData[ptr+1]=='?') && ptr<readResult-sizeof(serial_format)) {
			ptr++;
		}
		serial_format* ptr_to_first_valid = (serial_format*) &incomingData[ptr];

		printf("RecX:%d\n",ptr_to_first_valid->x);
		printf("RecY:%d\n",ptr_to_first_valid->y);
        //However always read serial data.
#ifdef (__linux__)
		int errNum = FFupdate(joy,ptr_to_first_valid->x);
#endif
    }
    SP->~Serial();
#ifdef __linux__
	SDL_JoystickClose(joy);
#endif
    
	system("pause");
	return 0;
}

int FFupdate( SDL_Joystick * joystick , unsigned short center) 
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
 effect.condition.center = center;//NEED CONVERSION!!!

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

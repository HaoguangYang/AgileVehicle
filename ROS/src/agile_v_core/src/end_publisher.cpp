#include "encoder.h"

ros::Publisher wheel_pub[4];
wheel_pub[0] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl00",2);
wheel_pub[1] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl01",2);
wheel_pub[2] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl02",2);
wheel_pub[3] = handle.advertise<std_msgs::UInt16MultiArray>("WheelControl03",2);

int main(double* steerVal, double* driveVal)
{
    uint16_t steer[4];
    uint16_t drive[4];
    uint16_t brake[4]
    std_msgs::UInt16MultiArray WheelCtrl[4];
	while (ros::ok())
	{
	    double ratio = PowerMizer();
	    bool IsBrake = true;
	    for (i=0; i<4; i++){
    		steer[i] = Encoder::reverseAngleLookup(steerVal[i]);
    		IsBrake = IsBrake && (driveVal[i]<=0);
    	}
    	if (IsBrake)
    	{
    	    for (i = 0; i<4; i++){
    	        brake[i] = -driveVal[i]*ratio;
    	        drive[i] = 0;
    	    }
    	}
    	else
    	{
    	    for (i = 0; i<4; i++){
    	        brake[i] = 0;
    	        drive[i] = driveVal[i]*ratio;
    	    }
    	}
    	
    	#pragma omp parallel for num_threads(4)
    	for (i = 0; i<4; i++){
        	WheelCtrl[i].data[0] = steer[i];
	    	WheelCtrl[i].data[1] = drive[i];
	    	WheelCtrl[i].data[2] = brake[i];
	    	WheelCtrl[i].data[3] = 0;
	    }
    	//publish control message
    	
        #pragma omp parallel for num_threads(4)
    	for (i = 0; i<4; i++){
        	wheel_pub[i].publish(WheelCtrl[i]);
	    }
	    
	    usleep(25000);
	    ros::spinOnce();
	}
}


#include "end_publisher.h"

uint16_t reverse_MotorPerformance(double Speed, double Torque)
{
	const double gain1 = 0.09549296586;
	const double gain2 = 10.33;		//N.m/V
	const double gain3 = 0.6;		//Friction-induced Torque
    double v_input = (Torque + gain3 + gain1*Speed)/gain2 + 1.2;
	return (v_input*51);	//*255/5
}

int main(double* steerVal, double* driveVal, double* Torque)
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
    	        brake[i] = reverse_MotorPerformance(-driveVal[i], -Torque[i]) * ratio;
    	        drive[i] = 0;
    	    }
    	}
    	else
    	{
    	    for (i = 0; i<4; i++){
    	        brake[i] = 0;
    	        drive[i] = reverse_MotorPerformance(driveVal[i], Torque[i]) * ratio;
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


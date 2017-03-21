#include "end_publisher.h"

uint16_t reverse_MotorPerformance(double Speed, double Torque)
{
	const double gain1 = 0.09549296586;
	const double gain2 = 10.33;		//N.m/V
	const double gain3 = 0.6;		//Friction-induced Torque
    double v_input = (Torque + gain3 + gain1*Speed)/gain2 + 1.2;
	return (v_input*51);	//*255/5
}

void publishToWheels(double* steerVal, double* driveVal, double* Torque)
{
    uint16_t steer[4];
    uint16_t drive[4];
    uint16_t brake[4];
    std_msgs::UInt16MultiArray WheelCtrl[4];
    ros::NodeHandle handle;
    ros::Publisher wheel_pub[4] = {handle.advertise<std_msgs::UInt16MultiArray>("WheelControl00",2), \
                                   handle.advertise<std_msgs::UInt16MultiArray>("WheelControl01",2), \
                                   handle.advertise<std_msgs::UInt16MultiArray>("WheelControl02",2), \
                                   handle.advertise<std_msgs::UInt16MultiArray>("WheelControl03",2)};
    for (int i=0; i<4; i++){
        WheelCtrl[i].layout.dim.push_back(std_msgs::MultiArrayDimension());
        WheelCtrl[i].layout.dim[0].label = "WheelControl";
        WheelCtrl[i].layout.dim[0].size = 4;
        WheelCtrl[i].layout.dim[0].stride = 1*4;
    }

    double ratio = 1.0; //PowerMizer();
    bool IsBrake = true;
    for (int i=0; i<4; i++){
   		steer[i] = Enc[0][i].reverseAngleLookup(steerVal[i]);
   		IsBrake = IsBrake && (driveVal[i]<=0);
   	}
   	if (IsBrake)
   	{
   	    for (int i = 0; i<4; i++){
   	        brake[i] = reverse_MotorPerformance(-driveVal[i], -Torque[i]) * ratio;
   	        drive[i] = 0;
   	    }
   	}
   	else
   	{
   	    for (int i = 0; i<4; i++){
   	        brake[i] = 0;
   	        drive[i] = reverse_MotorPerformance(driveVal[i], Torque[i]) * ratio;
   	    }
   	}
   	
   	#pragma omp parallel for num_threads(4)
   	for (int i = 0; i<4; i++){
       	WheelCtrl[i].data.push_back (steer[i]);
    	WheelCtrl[i].data.push_back (drive[i]);
    	WheelCtrl[i].data.push_back (brake[i]);
    	WheelCtrl[i].data.push_back (0);
    }
   	//publish control message
   	
    #pragma omp parallel for num_threads(4)
   	for (int i = 0; i<4; i++){
       	wheel_pub[i].publish(WheelCtrl[i]);
    }
    
	return;
}

//Publish to User or GUI the information of the vehicle.
void publishToUser()
{
    agile_v_core::kinematics VehicleKinematics;
	ros::NodeHandle handle;
	ros::Publisher kineStat = handle.advertise<agile_v_core::kinematics>("VehicleKinematics",5);
	
	VehicleKinematics.CM_Velocity[0] = Actual.speed[0];
	VehicleKinematics.CM_Velocity[1] = Actual.speed[1];
	VehicleKinematics.CM_AngularVel = Actual.omega;
	#pragma omp parallel for num_threads(4)
	for (int i = 0; i<4; i++)
    {
    	VehicleKinematics.Wheel_LinearVel[i] = Enc[1][i].extractDiff()*Vehicle.WheelRadius / step_time;
    	VehicleKinematics.Wheel_SteerAngl[i] = Enc[0][i].extractAngle();
    }
    VehicleKinematics.Time_Stamp = ros::Time::now();
    
    kineStat.publish(VehicleKinematics);
    return;
}

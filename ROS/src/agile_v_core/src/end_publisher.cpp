/*
 * Host-side communication interface between host and Single Wheel Driving Units.
 * This code is released under GPL-3.0 License.
 *
 * Author: Haoguang Yang
 * Mar 20, 2017
 *
 */

#include "end_publisher.h"

int reverse_MotorPerformance(double AngSpeed, double Torque)
{
	const double gain1 = 0.09549296586;
	const double gain2 = 10.33;		//N.m/V
	const double gain3 = 0.6;		//Friction-induced Torque
    double v_input = (Torque + gain3 + gain1*AngSpeed)/gain2 + 1.2;
	return (v_input*51);	//*255/5
}

void publishToWheels(ros::NodeHandle handle, ros::Publisher* wheel_pub, std_msgs::UInt16MultiArray* WheelCtrl, \
                     double* steerVal, double* driveVal, double* Torque)
{
    uint16_t steer[4];
    uint16_t drive[4];
    uint16_t brake[4];

    bool IsBrake = true;
    for (int i=0; i<4; i++){
   		steer[i] = Enc[0][i].reverseAngleLookup2x(steerVal[i]);
   		IsBrake = IsBrake && (driveVal[i]<=0);
   	}
   	if (IsBrake)
   	{
   	    for (int i = 0; i<4; i++){
   	        brake[i] = max(min(reverse_MotorPerformance(-driveVal[i], -Torque[i]), 255),0);
   	        drive[i] = 0;
   	    }
   	}
   	else
   	{
   	    for (int i = 0; i<4; i++){
   	        brake[i] = 0;
   	        drive[i] = max(min(reverse_MotorPerformance(driveVal[i], Torque[i]),255),0);
   	    }
   	}
   	
   	for (int i = 0; i<4; i++){
       	WheelCtrl[i].data[0] = steer[i];
    	WheelCtrl[i].data[1] = drive[i];
    	WheelCtrl[i].data[2] = brake[i];
    	WheelCtrl[i].data[3] = 0;
    }
    
    system("clear");
    for (int i = 0; i<4; i++){
    	cout << "Steering of Wheel " << i << " :    " << WheelCtrl[i].data[0] << endl;
    }
    
   	//publish control message
   	for (int i = 0; i<4; i++){
       	wheel_pub[i].publish(WheelCtrl[i]);
    }
	return;
}

//Publish to User or GUI the information of the vehicle.
void publishToUser(ros::NodeHandle handle, ros::Publisher kineStat)
{
    agile_v_core::kinematics VehicleKinematics;
	
	VehicleKinematics.CM_Velocity[0] = Actual.speed[0];
	VehicleKinematics.CM_Velocity[1] = Actual.speed[1];
	VehicleKinematics.CM_AngularVel = Actual.omega;
	
	for (int i = 0; i<4; i++)
    {
    	VehicleKinematics.Wheel_LinearVel[i] = Enc[1][i].extractDiff()*Vehicle.WheelRadius / step_time;
    	VehicleKinematics.Wheel_SteerAngl[i] = Enc[0][i].extractAngle();
    }
    VehicleKinematics.Time_Stamp = ros::Time::now();
    
    kineStat.publish(VehicleKinematics);
    return;
}

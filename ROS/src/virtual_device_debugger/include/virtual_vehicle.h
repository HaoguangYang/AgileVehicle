#ifndef VIRTUAL_VEHICLE_H
#define VIRTUAL_VEHICLE_H
#include <ros/ros.h>
#include <sys/time.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <math.h>

extern double ctrlVolt[4];
extern double AngSpeed[4];
extern float vel[4];
extern float angle_real[4];

void naive_driving_controller(int i, const std_msgs::UInt16MultiArray& ctrl_var);

void modeled_driving_controller(int i, const std_msgs::UInt16MultiArray& ctrl_var);

void Actuate0( const std_msgs::UInt16MultiArray& ctrl_var);
void Actuate1( const std_msgs::UInt16MultiArray& ctrl_var);
void Actuate2( const std_msgs::UInt16MultiArray& ctrl_var);
void Actuate3( const std_msgs::UInt16MultiArray& ctrl_var);

void Flip(bool direc, uint8_t which_one);
void Steering(int i);
void Throttling(int i);
void Query(int i);
void Publish(int i, ros::Publisher* assessActual, ros::Publisher* assessPower);
void setup(void);
void loop(ros::Publisher* assessActual, ros::Publisher* assessPower);

int sim_vehicle(int argc, char* argv[]);

int main(int argc, char* argv[]);

#endif

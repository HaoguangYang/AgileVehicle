#ifndef GLOBALS_H
#define GLOBALS_H

typedef struct VehiclePhysicalParams {
	double TrackWidth; // 左右轮间距
	double WheelBase;  // 前后轴距
	double Mass;       // 质量
	double WheelRadius; // 轮子半径
};

typedef struct Kinematic{
	double speed[2]; // 固连车辆坐标系，xy
	double omega;
	double accel[2];
	double angularAccel;
};

class ElectricStat{                 //Wheel Power Monitor
public:
	float _volt;
	float _ampS;
	float _ampD;
	
	float getUnitPwr(){ return (_volt*(_ampS+_ampD));}
	
	float getUnitCur(){	return (_ampS + _ampD);}
	
	float getUnitVolt(){ return (_volt);}
};

extern double step_time;            //Arduino refresh time

extern Encoder Enc[2][4];           //[0][*] - Steering; [1][*] - Driving
extern Kinematic Actual;
extern double steerVal[4];          // 四个轮子转弯角度
extern double driveVal[4];          // 四个轮子线速度
extern double Torque[4];            // 四个轮子扭矩
// 左前，右前，左后，右后
extern VehiclePhysicalParams Vehicle;   //Vehicle size, mass, etc.

extern ElectricStat ElectricMon[4]; //4-Wheel Power Monitors
extern bool IsZeroCorrect[4];       //Steering encoder zeros readiness

#endif

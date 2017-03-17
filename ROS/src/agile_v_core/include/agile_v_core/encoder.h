#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <math.h>
class Encoder{
public:
	// 构造函数
	Encoder(int zero, int resolution=4096):_zero(zero), _resolution{};
	// 成员函数
    // get 函数
    const int getResolution(){ return _resolution;}
	const int getZero() {return _zero;}
	int getAngleInt() {return _angle_int;}
	double getAngleRad() { return _angle_rad;}
	int getDiff(){ return (_angle_int - _angle_int_old)%_resolutioin;}  // 返回两次的差值
	// 90度提醒
	void update(uint16_t raw_input) {
		_angle_int_old = _angle_int; // 把旧值赋回去

		_angle_int = ((int)raw_input - _zero + _resolution)%_resolution;
		// 解决转向圈数
		if(getDiff() > _resolution/2)
		    if(!_round)
			    std::cout << "CAUTION!!!!!Error steering angle!!!";
			else
			    _round = false;
		
		if(getDiff() < -_resolution/2)
		    if(_round)
			    std::cout << "CAUTION!!!!!Error steering angle!!!";
			else
			    _round = true;
	    _angle_rad = M_PI*_angle_int/_resolution;
	}
	
private:
    // const value
    const int _resolution;
    const int _zero; //零点
	bool _round; // if true 正转一圈的范围内, if false 反转一圈的范围内
	int _angle_int;
	double _angle_rad;   // 0~pi
	
	int _angle_int_old; // 上一次的角度
};

#endif

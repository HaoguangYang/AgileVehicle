#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <math.h>
#include <string>
#include <stdint.h>
using namespace std;

class Encoder {

private:
    uint16_t        _resolution;
    uint16_t        _zero;
    uint16_t        _lastMark;
    int32_t         _cycle_old;
    bool            _noNewData;
    int16_t          _diff_old;
    
public:
    
    uint16_t _value;
    int32_t _cycle;
    
    // 构造函数
    Encoder(uint16_t resolution=4096, int32_t cycle=0, bool noNewData=true):_resolution(resolution),_cycle(cycle),_noNewData(noNewData){
        _value = 0;
        _cycle = 0;
    }
    
    Encoder(uint16_t res, bool noNewData=true):_noNewData(noNewData){
        _resolution = res;
        _value = 0;
        _cycle = 0;
    }
    
    void setRes(uint16_t res){
        _resolution = res;
    }
    
    uint16_t getRes(){
        return (_resolution);
    }
    
    void setZero(uint16_t zero){
        _zero = zero;
    }
    
    uint16_t rectify(uint16_t encoderIn)
    {
	    return ((encoderIn - _zero + _resolution) % _resolution);		//Convert to 0~4095 loop
    }
    
    void mark(){
        _lastMark = _value;
        _cycle_old = _cycle;
    }
	
	void update(uint16_t NewData)
	{
	    mark();
		_value = rectify(NewData);
        if (_value-_lastMark<-_resolution/2)    //From 11*** to 00***, cycle + 1
            ++_cycle;
        if (_value-_lastMark>_resolution/2)
            --_cycle;
        _noNewData = false;
		return;
	}
	
	void update_value(uint16_t NewData)
	{
		_value = rectify(NewData);
        if (_value-_lastMark<-_resolution/2)    //From 11*** to 00***, cycle + 1
            ++_cycle;
        if (_value-_lastMark>_resolution/2)
            --_cycle;
        _noNewData = false;
		return;
	}
	
	double extractDiff()
	{
		int16_t tmpValue;
		if (_noNewData){
		    tmpValue = _diff_old;
		}
        else{
            if (_value-_lastMark<-_resolution/2){    //From 11*** to 00***, cycle + 1
                tmpValue = _value - _lastMark + (_cycle-_cycle_old)*_resolution;
            }
            if (_value-_lastMark>_resolution/2){
                tmpValue = _value - _lastMark + (_cycle-_cycle_old)*_resolution;
            }
            _diff_old = tmpValue;
            mark();
        }
        _noNewData = true;
		return (2*M_PI*(tmpValue)/_resolution);
	}
    
    double extractAngle()
    {
        return (2.0*M_PI*(_value/(double)_resolution+_cycle));    //Return angle in RAD.
    }
    
    double extractAngle_OneCycle()
    {
        return (2.0*M_PI*_value/(double)_resolution);
    }
    
    uint16_t reverseAngleLookup(double angle_in)
    {
        return (((uint16_t)(angle_in*_resolution*0.5/M_PI))%_resolution);
    }
    
    uint16_t reverseAngleLookup2x(double angle_in)
    {
        return (((uint16_t)(angle_in*_resolution/M_PI))%_resolution);
    }
};

#endif


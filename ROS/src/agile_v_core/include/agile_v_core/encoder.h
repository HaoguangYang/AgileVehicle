#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <math.h>
#include <string>
#include <stdint.h>
using namespace std;

class Encoder {

private:
    string          _label;
    unsigned short  _resolution;
    unsigned short  _zero;
    
public:
    uint16_t _lastMark;
    uint16_t _value;
    int32_t _cycle;
    
    Encoder(){
        _label = "newEncoder";
        _resolution = 4096;
        _cycle = 0;
        _value = 0;
    }
    
    Encoder(const char *label, unsigned short res){
        _label = label;
        _resolution = res;
        _cycle = 0;
        _value = 0;
    }
    
    Encoder(string& label, unsigned short res){
        _label = label;
        _resolution = res;
        _cycle = 0;
        _value = 0;
    }
    
    void setLabel(const char *label){
        _label = label;
    }
    
    void setLabel(string& label){
        _label = label;
    }
    
    void setRes(unsigned short res){
        _resolution = res;
    }
    
    void setZero(unsigned short zero){
        _zero = zero;
    }
    
    uint16_t rectify(uint16_t encoderIn)
    {
	    return ((encoderIn - _zero + _resolution) % _resolution);		//Convert to 0~4095 loop
    }
    
    void mark(){
        _lastMark = _value;
    }
	
	void update(uint16_t NewData)
	{
		_value = rectify(NewData);
        if (_value-_lastMark<-_resolution/2)    //From 11*** to 00***, cycle + 1
            ++_cycle;
        if (_value-_lastMark>_resolution/2)
            --_cycle;
        mark();
		return;
	}
	
	double extractDiff(uint16_t NewData)
	{
		_value = rectify(NewData);
		int16_t tmpValue;
        if (_value-_lastMark<-_resolution/2){    //From 11*** to 00***, cycle + 1
            ++_cycle;
            tmpValue = _value - _lastMark + _resolution;
        }
        if (_value-_lastMark>_resolution/2){
            --_cycle;
            tmpValue = _value - _lastMark - _resolution;
        }
        mark();
		return (2*M_PI*(tmpValue)/_resolution);
	}
    
    double extractAngle()
    {
        return (2*M_PI*(_value/_resolution+_cycle));                            //Return angle in RAD.
    }
    
    double extractAngle_OneCycle()
    {
        return (2*M_PI*_value/_resolution);
    }
    
    uint16_t reverseAngleLookup(double angle_in)
    {
        return (((uint16_t)(angle_in*_resolution*0.5/M_PI))%_resolution);
    }
};

#endif


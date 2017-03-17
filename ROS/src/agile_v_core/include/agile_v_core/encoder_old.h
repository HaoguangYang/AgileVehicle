#ifndef ENCODE_H
#define ENCODE_H

#include <string>
#include <math.h>

#define pi M_PI
using namespace std;
//const double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862;

// 从encoder中读出信息
// 还原成角速度，和角度
class Encoder {

private:
    string          _label;
    unsigned short  _resolution;
    unsigned short  _symbol;
    unsigned short  _zero;
    
public:
    int16_t _last_mark; // 编码器上次读回来的数值
    int16_t _value;     // 带符号的

    //Bits not taken for encoder data stored in int16_t
    Encoder():_label("newEncoder"),_resolution(4096),_symbol(4),_value(0){}
    
    Encoder(const char *label, unsigned short resolution):_label(label), _value(0),
						          _resolution(resolution){
        unsigned int i = 0;
        do {
            ++i;
        } while ((res >> 1));
        _symbol = 16-i;
    }
    
    Encoder(string &label, unsigned short resolution):_label(label), _value(0),
						          _resolution(resolution){
        unsigned int i = 0;
        do {
            ++i;
        } while ((res >> 1));
        _symbol = 16-i;
    }
    
    void setLabel(const char *label){
        _label = label;
    }
    
    void setLabel(string& label){
        _label = label;
    }
    
    void setRes(unsigned short res){
        _resolution = res;
        unsigned int i = 0;
        do {
            ++i;
        } while ((res >> 1));
        _symbol = 16-i;
    }
    
    // 设置零点
    void setZero(unsigned short zero){
        _zero = zero;
    }
    
    uint16_t rectify(uint16_t encoder_in) // 从编码器读入
    {
	    return (encoder_in - _zero)%4096;	//Convert to 0~4095 loop
    }
    
    void mark(){ // 刷新旧值
        _last_mark = _value;
    }
	
	void update(uint16_t new_data) // 从编码器读进新值
	{
		uint16_t encoder_in = rectify(new_data);
		int8_t flag = _value/_resolution; // 编码器转的圈数 ???
        if (((Encoder_in)<<(_symbol+1))>>(16-_symbol+1)==0 && ((last_mark)<<(_symbol+1))>>(16-_symbol+1)==3)    //From 11*** to 00***, flag + 1
            ++flag;
        if (((Encoder_in)<<(_symbol+1))>>(16-_symbol+1)==3 && ((last_mark)<<(_symbol+1))>>(16-_symbol+1)==0)
            --flag;
        
        value = (int16_t)(rectify(Encoder_in) + ((uint16_t)((flag)>>7))<<15 + (((uint16_t)flag)<<(16-_symbol))>>1);
        mark();
		return;
	}
	
	double extractDiff(uint16_t NewData)
	{
		Encoder_in = rectify(NewData);
		int8_t flag = value>>(16-_symbol-1);
        if (((Encoder_in)<<(_symbol+1))>>(16-_symbol+1)==0 && ((last_mark)<<(_symbol+1))>>(16-_symbol+1)==3)    //From 11*** to 00***, flag + 1
            ++flag;
        if (((Encoder_in)<<(_symbol+1))>>(16-_symbol+1)==3 && ((last_mark)<<(_symbol+1))>>(16-_symbol+1)==0)
            --flag;
        
        value = (int16_t)(rectify(Encoder_in) + ((uint16_t)((flag)>>7))<<15 + (((uint16_t)flag)<<(16-_symbol))>>1);	//Reading + Sign + Flag
        int16_t tmp_value = value - last_mark;
        mark();
		return (2*pi*(tmp_value)/_resolution);
	}
    
    double extractAngle()
    {
        return (2*pi*value/_resolution);                            //Return angle in RAD.
    }
    
    double extractAngle_OneCycle()
    {
        return (2*pi*(((uint16_t)value<<(_symbol+1))>>(_symbol+1))/_resolution);
    }
    
    double extractAngle_TwoCycle()
    {
        return (2*pi*(((uint16_t)value<<(_symbol))>>(_symbol))/_resolution);
    }
    
    double extractAngle_FourCycle()
    {
        return (2*pi*(((uint16_t)value<<(_symbol-1))>>(_symbol-1))/_resolution);
    }
    
    uint16_t reverseAngleLookup(double angle_in)
    {
        return ((((uint16_t)(angle_in*_resolution/(2*pi)))<<(_symbol+1))>>(_symbol+1));
    }
};
#endif


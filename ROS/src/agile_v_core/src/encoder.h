#include <string>
using namespace std;
double pi = 3.1415926535897932384626433832795028841971693993751058209749445923078164062862;

class Encoder {

private:
    string          _label;
    unsigned short  _resolution;
    unsigned short  _symbol;
    unsigned short  _zero;
    
public:
    int16_t last_mark;
    int16_t value;
    
    Encoder(){
        _label = "newEncoder";
        _resolution = 4096;
        _symbol = 4;                            //Bits not taken for encoder data stored in int16_t
        value = 0;
    }
    
    Encoder(const char *label, unsigned short res){
        _label = label;
        _resolution = res;
        unsigned int i = 0;
        do {
            ++i;
        } while ((res >> i));
        _symbol = 16-i;
        value = 0;
    }
    
    Encoder(string& label, unsigned short res){
        _label = label;
        _resolution = res;
        unsigned int i = 0;
        do {
            ++i;
        } while ((res >> i));
        _symbol = 16-i;
        value = 0;
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
        } while ((res >> i));
        _symbol = 16-i;
    }
    
    void setZero(unsigned short zero){
        _zero = zero;
    }
    
    uint16_t rectify(uint16_t Encoder_in)
    {
	    return (((Encoder_in - _zero)<<(_symbol+1))>>(_symbol+1));		//Convert to 0~4095 loop
    }
    
    void mark(){
        last_mark = value;
    }
	
	double extractDiff(uint16_t Encoder_in)
	{
		int8_t flag = value>>(16-_symbol-1);
        if (((Encoder_in)<<(_symbol+1))>>(16-_symbol+1)==0 && ((last_mark)<<(_symbol+1))>>(16-_symbol+1)==3)    //From 11*** to 00***, flag + 1
            ++flag;
        if (((Encoder_in)<<(_symbol+1))>>(16-_symbol+1)==3 && ((last_mark)<<(_symbol+1))>>(16-_symbol+1)==0)
            --flag;
        
        value = (int16_t)(rectify(Encoder_in) + ((uint16_t)((flag)>>7))<<15 + (((uint16_t)flag)<<(16-_symbol))>>1);
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


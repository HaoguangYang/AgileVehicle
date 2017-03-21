#include <ros.h>
#include <FlexiTimer2.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#define CONTRL      3
#define BACK        4
#define BREAK       5 //connecting to driving motor controller.
#define csn        13 //yellow
#define datS        2 //green
#define datD        8 //green
#define clk         9 //blue
#define PUL         6
#define DIR         7 // direction for the angle motor
#define VOLT        A3
#define AMPD        A7
#define AMPS        A6

ros::NodeHandle handle;
int encoder_resolution = 4095;
int pulseTime = 100;
int updateTime = 40;
int Angle = 0;
uint16_t steeringTarget = ActuatorStatus.data[0];

std_msgs::UInt16MultiArray ActuatorStatus;
/************
    unsigned short StrActual;
    unsigned short DrvActual;
************/
std_msgs::Float32MultiArray PowerStatus;
/************
    float Voltage;
    float CurrentS;
    float CurrentD;
************/
/************
std_msgs::UInt16MultiArray ctrl_var;
    int inputSteer;
    int inputDrive;
    int inputBreak;
    int reverse;
************/

//***MODIFY UNIT-SPECIFIC TOPICS AS NECESSARY!!!***//
ros::Publisher assessActual("WheelActual01", &ActuatorStatus);
ros::Publisher assessPower("UnitPower01", &PowerStatus);

void Actuate( const std_msgs::UInt16MultiArray& ctrl_var){
	if(ctrl_var.data[3]>0){ // 倒车
        digitalWrite(BACK,HIGH);
	}//adjust for the switch
    else{
		digitalWrite(BACK,LOW);
	}
	if (ctrl_var.data[2]==0){
		ctrl_var.data[1] = ctrl_var.data[1]*175/255+80;	//Calibration of controller to elliminate dead zone
		analogWrite(CONTRL,ctrl_var.data[1]);	//Normal Driving
		analogWrite(BREAK,0);
	}
	else{											//Breaking
		analogWrite(CONTRL,0);
		analogWrite(BREAK,ctrl_var.data[2]);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget = ctrl_var.data[0];
}

void Steering(){
	//-------------------start angle control------------------------------------
	if(steeringTarget < 0 || steeringTarget >encoder_resolution) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
		int err = min(min(abs(steeringTarget-Angle),abs(steeringTarget-Angle+encoder_resolution+1)),abs(steeringTarget-Angle-encoder_resolution-1));
        if(!(err<15)) {
			pulseTime = 55000/(err+500);	//Need Modification
            if (steeringTarget>Angle) {
                OneUp(pulseTime,0);
            }
            else {
                OneUp(pulseTime,1);
            }  
        }
		FlexiTimer2::set(pulseTime*2,Steering);  //time in the unit of ms
        //end while loop 
    }
	//----------------end angle control---------------------------------------------  
}

//***MODIFY UNIT-SPECIFIC TOPICS AS NECESSARY!!!***//
ros::Subscriber<std_msgs::UInt16MultiArray> sub("WheelControl01", &Actuate);

void setup() {
	// set driving control
	pinMode(DIR, OUTPUT);
	pinMode(PUL, OUTPUT);
	// set angle encoder
	pinMode(csn, OUTPUT);
	pinMode(datS, INPUT);
	pinMode(datD, INPUT);
	pinMode(clk, OUTPUT);
	
	handle.initNode();
	handle.subscribe(sub);
	
    //PowerStatus.layout.dim_length = 1;
    malloc(sizeof(std_msgs::MultiArrayDimension) * 3);
    PowerStatus.layout.dim[0].label = "UnitPower";
    PowerStatus.layout.dim[0].size = 3;
    PowerStatus.layout.dim[0].stride = 1*3;
    //PowerStatus.layout.data_offset = 0;
    PowerStatus.data = (float *)malloc(sizeof(float)*3);
    PowerStatus.data_length = 3;
    handle.advertise(assessPower);
    
    //ActuatorStatus.layout.dim_length = 1;
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    ActuatorStatus.layout.dim[0].label = "WheelActual";
    ActuatorStatus.layout.dim[0].size = 2;
    ActuatorStatus.layout.dim[0].stride = 1*2;
    //ActuatorStatus.layout.data_offset = 0;
    ActuatorStatus.data = (uint16_t *)malloc(sizeof(uint16_t)*2);
    ActuatorStatus.data_length = 2;
    handle.advertise(assessActual);
	
	steeringTarget = ActuatorStatus.data[0];	//Stop Init Steering of the wheel
	
	//Initiate Steering Actuator and run via FlexiTimer
	Steering();
	FlexiTimer2::set(pulseTime*2,Steering);  //time in the unit of ms
    FlexiTimer2::start();
}

int data = 0;
int counter = 0;

void loop() {
    Query();
	handle.spinOnce();
	delay(updateTime);
}

// the function to move the angle motor for once
void OneUp(unsigned int time, bool direc) {
  if (direc == 0){
    digitalWrite(DIR,HIGH);
  }
  else if (direc == 1) {
    digitalWrite(DIR, HIGH);
  }
  // give a pulse
  digitalWrite(PUL,HIGH);
  delayMicroseconds(time);
  digitalWrite(PUL,LOW);
  delayMicroseconds(time);
   //unit of time is us
  delay(2);
}

void Query()
{
  uint16_t dataActuator[2] = {0};
  float dataPower[3];
  digitalWrite(csn,LOW);
  delayMicroseconds(1);
  for (int k=0;k<12;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
    dataActuator[0]=(dataActuator[0]<<1)+digitalRead(datS);		//Steering
    dataActuator[1]=(dataActuator[1]<<1)+digitalRead(datD);		//Driving
    //delayMicroseconds(1);
  }
  for (int k=0;k<6;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
  }
  digitalWrite(csn,HIGH);
  Angle=dataActuator[0];
  
  dataPower[0] = analogRead (VOLT)*(0.02892+0.00002576*analogRead (VOLT))+2.99;
  dataPower[1] = (analogRead (AMPS)-512)*30/409.6;
  dataPower[2] = (analogRead (AMPD)-512)*30/409.6;
  ActuatorStatus.data = dataActuator;
  PowerStatus.data = dataPower;
  assessActual.publish (&ActuatorStatus);
  assessPower.publish (&PowerStatus);
  //Serial.write((const uint8_t*)&to_send,sizeof(serial_format));
}


#include <ros.h>
#include <FlexiTimer2.h>
#include <std_msgs/Empty.h>
#include "struct.h"
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
int encoder_resolution = 4096;
String inputString = "";
String inputAngle  = "";
int pulseTime = 100;
int angleTime = 100;
int Angle = 0;

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

ros::Publisher assessActual("WheelActual", &ActuatorStatus);
ros::Publisher assessPower("UnitPower", &PowerStatus);

void Actuate( const std_msgs::Int32MultiArray& ctrl_var){
	if(ctrl_var.data[4]>0){ // 倒车
        digitalWrite(BACK,HIGH);
	}//adjust for the switch
    else{
		digitalWrite(BACK,LOW);
	}
	if (ctrl_var.data[3]==0){
		analogWrite(CONTRL,ctrl_var.data[2]);	//Normal Driving
	}
	else{											//Breaking
		analogWrite(CONTRL,0);
		digitalWrite(BREAK,HIGH);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
    //-------------------start angle control------------------------------------
    if(ctrl_var.data[1] < 0 || ctrl_var.data[1] >encoder_resolution) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
        if(!(abs(ctrl_var.data[1]-Angle)<40 ||abs(ctrl_var.data[1]-Angle+encoder_resolution)<40||abs(ctrl_var.data[1]-Angle-encoder_resolution)<40)) {
            if (ctrl_var.data[1]>Angle) {
                OneUp(pulseTime,0);
            }
            else {
                OneUp(pulseTime,1);
            }  
        }
        //end while loop 
    }
    //----------------end angle control---------------------------------------------  
}
ros::Subscriber<std_msgs::Int32MultiArray> sub("WheelControl", &ctrl_var);

void setup() {
	Serial.begin(9600);
	inputString.reserve(100);
	inputAngle.reserve(100);
	// set driving control
	pinMode(DIR, OUTPUT);
	pinMode(PUL, OUTPUT);
	// set angle encoder
	pinMode(csn, OUTPUT);
	pinMode(dat, INPUT);
	pinMode(clk, OUTPUT);
	
	handle.initNode();
	handle.subscribe(sub);
	handle.advertise(assessActual);
	
	Query();
	FlexiTimer2::set(angleTime, Query);
	FlexiTimer2::start();
}

int data = 0;
int counter = 0;

void loop() {
	handle.spinOnce();
	delay(1);
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
  unsigned short dataActuator[2];
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
  
  dataPower[0] = 0.02892*analogRead (VOLT)*(1+0.0008907*analogRead (VOLT))+2.99;
  dataPower[1] = (analogRead (AMPS)-512)*30/409.6;
  dataPower[2] = (analogRead (AMPD)-512)*30/409.6;
  ActuatorStatus.data = dataActuator;
  PowerStatus.data = dataPower;
  assessActual.publish (&ActuatorStatus);
  assessPower.publish (&PowerStatus);
  //Serial.write((const uint8_t*)&to_send,sizeof(serial_format));
}

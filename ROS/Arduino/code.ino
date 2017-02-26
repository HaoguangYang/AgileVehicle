#include <ros.h>
#include <FlexiTimer2.h>
#include <std_msgs/Empty.h>
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

typedef struct {
  char end_1;
  char end_2;
  unsigned short StrActual;
  unsigned short DrvActual;
  float Voltage;
  float CurrentS;
  float CurrentD;
} to_send;
typedef struct {
  int inputSteer;
  int inputDrive;
  int inputBreak;
  bool reverse;
} ctrl_var;

ros::Publisher assessActual("WheelActual", &to_send);

void Actuate( const ctrl_var& control_msg){
	if(control_msg.reverse){ // 倒车
        digitalWrite(BACK,HIGH);
	}//adjust for the switch
    else{
		digitalWrite(BACK,LOW);
	}
	if (!inputBreak>0){
		analogWrite(CONTRL,control_msg.inputDrive);	//Normal Driving
	}
	else{											//Breaking
		analogWrite(CONTRL,0);
		digitalWrite(BREAK,HIGH);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
    //-------------------start angle control------------------------------------
    if(control_msg.inputSteer < 0 || control_msg.inputSteer >encoder_resolution) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
        if(!(abs(control_msg.inputSteer-Angle)<40 ||abs(control_msg.inputSteer-Angle+encoder_resolution)<40||abs(control_msg.inputSteer-Angle-encoder_resolution)<40)) {
            if (control_msg.inputSteer>Angle) {
                OneUp(pulseTime,1);
            }
            else {
                OneUp(pulseTime,2);
            }  
        }
        //end while loop 
    }
    //----------------end angle control---------------------------------------------  
}
ros::Subscriber<std_msgs::Empty> sub("WheelControl", &Actuate );

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
void OneUp(unsigned int time, unsigned int direc) {
  if (direc == 1){
    digitalWrite(DIR,HIGH);
  }
  else if (direc == 2) {
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
  dataS=0;
  dataD=0;
  digitalWrite(csn,LOW);
  delayMicroseconds(1);
  for (int k=0;k<12;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
    valS=digitalRead(datS);
    valD=digitalRead(datD);
    dataS=(dataS<<1)+valS;
    dataD=(dataD<<1)+valD;
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
  Angle=dataS;
  
  serial_format to_send;
  to_send.end_1=0x3f;
  to_send.end_2=0x3f;
  to_send.Steer=dataS;   //Steering
  to_send.Drive=dataD;   //Motor Speed
  to_send.Voltage = analogRead(VOLT);
  to_send.CurrentD = analogRead(AMPD);
  to_send.CurrentS = analogRead(AMPS);
  assessActual.publish (&to_send);
  //Serial.write((const uint8_t*)&to_send,sizeof(serial_format));
}

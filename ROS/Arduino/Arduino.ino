#include <ros.h>
//#include <FlexiTimer2.h>
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
const uint16_t encoder_resolution = 4095;
const int updateTime = 40;
int pulseTime = 5;                //Time in ms
uint16_t Angle = 0;
uint16_t steeringTarget = 0;

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
ros::Publisher assessActual("WheelActual00", &ActuatorStatus);
ros::Publisher assessPower("UnitPower00", &PowerStatus);

unsigned long time_last;                 //for Buffer flushing
unsigned long time_last_query;           //for Query
unsigned long time_now;


void Actuate( const std_msgs::UInt16MultiArray& ctrl_var){
    //Send Signals to stepper motor and BLDC according to messages subscribed
	if(ctrl_var.data[3]>0){ // 倒车
        digitalWrite(BACK,HIGH);
	}//adjust for the switch
    else{
		digitalWrite(BACK,LOW);
	}
	if (ctrl_var.data[2]==0){
		ctrl_var.data[1] = ctrl_var.data[1];	//Calibration of controller to elliminate dead zone
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

//***MODIFY UNIT-SPECIFIC TOPICS AS NECESSARY!!!***//
ros::Subscriber<std_msgs::UInt16MultiArray> sub("WheelControl00", &Actuate);

void Steering(){
	//-------------------start angle control------------------------------------
	if(steeringTarget < 0 || steeringTarget >encoder_resolution) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
		int err = min(min(abs(steeringTarget-Angle),abs(steeringTarget-Angle+encoder_resolution+1)),abs(steeringTarget-Angle-encoder_resolution-1));
        if(!(err<15)) {
			//pulseTime = 55000/(err+400);	//Need Modification
            if ((steeringTarget-Angle+encoder_resolution+1)%(encoder_resolution+1)<(encoder_resolution+1)*0.5) {
                Flip(0);
            }
            else {
                Flip(1);
            }  
        }
        //end while loop 
    }
	//----------------end angle control---------------------------------------------  
}

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
	//Query();
	//FlexiTimer2::set(updateTime,Query);  //time in the unit of ms
    //FlexiTimer2::start();
}

void loop() {
   time_now = millis();
   if (time_now - time_last > pulseTime){
       Steering();
       time_last = time_now;
   }
   if (time_now - time_last_query > updateTime){
       Query();
       time_last_query = time_now;
   }
   handle.spinOnce();
}

// the function to determine the wave pattern to servo
int V_last = LOW;
void Flip(bool direc) {
  if (direc == 0 && V_last == LOW){
    digitalWrite(DIR,HIGH);
    delayMicroseconds(1);
  }
  else if (direc == 1 && V_last == LOW) {
    digitalWrite(DIR, LOW);
    delayMicroseconds(1);
  }
  // give a pulse
  if (V_last == LOW){
    V_last = HIGH;
    digitalWrite(PUL,HIGH);
    delayMicroseconds(1);
  }
  else{
    V_last = LOW;
    digitalWrite(PUL,LOW);
    delayMicroseconds(1);
  }
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


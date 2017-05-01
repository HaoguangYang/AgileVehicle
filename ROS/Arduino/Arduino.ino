#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Int32MultiArray.h>

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
const uint16_t encoder_resolution = 4096;
const uint16_t         updateTime = 40000;	//Time in us
const uint16_t          queryTime = 1000;	//Time in us
uint16_t                pulseTime = 100;    //Time in us
uint16_t                    Angle = 0;
uint16_t           steeringTarget = 0;
const uint16_t _zero=0; 
uint16_t drive_input;
float throttle=1.0;

uint16_t dataActuator[2];
float dataPower[3];

//int16_t  Speed = 0;
//uint16_t _last = 0;

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
ros::Publisher assessActual("WheelActual0x", &ActuatorStatus);
ros::Publisher assessPower("UnitPower0x", &PowerStatus);

unsigned long time_last;                 //for Buffer flushing
unsigned long time_last_query;           //for Query
unsigned long time_last_publish;		 //for Publish

void Actuate( const std_msgs::UInt16MultiArray& ctrl_var){
    //Send Signals to stepper motor and BLDC according to messages subscribed
	if(ctrl_var.data[3]>0){ // 倒车
        digitalWrite(BACK,HIGH);
	}//adjust for the switch
    else{
		digitalWrite(BACK,LOW);
	}
	drive_input = ctrl_var.data[1];
	if (ctrl_var.data[2]==0){
		//ctrl_var.data[1] = ctrl_var.data[1] + 75;	//Calibration of controller to elliminate dead zone
		analogWrite(BREAK,0);
		analogWrite(CONTRL,drive_input);//*throttle);	//Normal Driving
	}
	else{											//Breaking
		analogWrite(CONTRL,0);
		analogWrite(BREAK,ctrl_var.data[2]);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget = encoder_resolution-1-ctrl_var.data[0];
}

//***MODIFY UNIT-SPECIFIC TOPICS AS NECESSARY!!!***//
ros::Subscriber<std_msgs::UInt16MultiArray> sub("WheelControl0x", &Actuate);

void Steering(){
	//-------------------start angle control------------------------------------
	if(steeringTarget > encoder_resolution-1) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
		int16_t err = (steeringTarget-Angle+(uint16_t)(0.5*encoder_resolution))%(encoder_resolution)-(encoder_resolution*0.5);
		//min(min(abs(steeringTarget-Angle),abs(steeringTarget-Angle+encoder_resolution)),abs(steeringTarget-Angle-encoder_resolution));
        if(!(abs(err)<40)) {
			pulseTime = 7000/(1.1*abs(err)+5);	//Need Modification
            if (err<0) {
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

void Throttling(){
    throttle = min(throttle * 1050.0 / max(PowerStatus.data[0]*PowerStatus.data[2],0.001),1.0);
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
    PowerStatus.layout.data_offset = 0;
    PowerStatus.data = (float *)malloc(sizeof(float)*3);
    PowerStatus.data_length = 3;
    handle.advertise(assessPower);
    
    //ActuatorStatus.layout.dim_length = 1;
    malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    ActuatorStatus.layout.dim[0].label = "WheelActual";
    ActuatorStatus.layout.dim[0].size = 2;
    ActuatorStatus.layout.dim[0].stride = 1*2;
    ActuatorStatus.layout.data_offset = 0;
    ActuatorStatus.data = (uint16_t *)malloc(sizeof(uint16_t)*2);
    ActuatorStatus.data_length = 2;
    handle.advertise(assessActual);
	
	steeringTarget = ActuatorStatus.data[0];	//Stop Init Steering of the wheel
	
}

void loop() {
   unsigned long time_now = micros();
   if ((unsigned long)(time_now - time_last) > pulseTime){
       Steering();
       time_last = micros();
   }
   if ((unsigned long)(time_now - time_last_query) > queryTime){
       Query();
       Throttling();
       time_last_query = micros();
   }
   if ((unsigned long)(time_now - time_last_publish) > updateTime){
	   Publish();
	   time_last_publish = micros();
   }
   handle.spinOnce();
}

// the function to determine the wave pattern to servo
bool V_last = false;
void Flip(bool direc) {
  if (!direc && !V_last){
    digitalWrite(DIR,HIGH);
  }
  else if (direc && !V_last) {
    digitalWrite(DIR, LOW);
  }
  // give a pulse
  V_last = !V_last;
  digitalWrite(PUL,V_last);
}

void Query()
{
  dataActuator[0] = 0;
  dataActuator[1] = 0;
  digitalWrite(csn,LOW);
  delayMicroseconds(1);
  for (uint8_t k=0;k<12;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
    dataActuator[0]=(dataActuator[0]<<1)+digitalRead(datS);		//Steering
    dataActuator[1]=(dataActuator[1]<<1)+digitalRead(datD);		//Driving
    //delayMicroseconds(1);
  }
  for (uint8_t k=0;k<6;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
  }
  digitalWrite(csn,HIGH);
  Angle=(dataActuator[0]-_zero+encoder_resolution)%encoder_resolution;
  dataActuator[0] = Angle;
  //Speed=(-dataActuator[1]+_last+encoder_resolution)%encoder_resolution;
  //_last = dataActuator[1];
  //Angle=dataActuator[0];
  
  dataPower[0] = analogRead (VOLT)*(0.02892+0.00002576*analogRead (VOLT))+2.99;
  dataPower[1] = (analogRead (AMPS)-512)*30/409.6;
  dataPower[2] = (analogRead (AMPD)-512)*30/409.6;
}

void Publish(){
    ActuatorStatus.data = dataActuator;
    PowerStatus.data = dataPower;
    assessActual.publish (&ActuatorStatus);
    assessPower.publish (&PowerStatus);
}


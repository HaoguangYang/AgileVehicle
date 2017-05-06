#include <ros/ros.h>
//#include <FlexiTimer2.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

/*
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
*/

ros::NodeHandle handle;
const uint16_t encoder_resolution = 4096;
const unsigned int updateTime = 40000;	                    //Time in us
int pulseTime[4] = {100, 100, 100, 100};                	//Time in us
uint16_t Angle[4] = {0,0,0,0};
float angle_real[4]={0., 0., 0., 0.};
int16_t Speed[4] = {0,0,0,0};
uint16_t steeringTarget[4] = {0,0,0,0};
uint16_t _zero[4] = {231,2646,1895,2297};
uint16_t _last[4] = {0,0,0,0};
uint16_t drive_input[4];
uint16_t drive_val[4];
float vel[4]={0.,0.,0.,0.};

std_msgs::UInt16MultiArray ActuatorStatus[4];
/************
    unsigned short StrActual;
    unsigned short DrvActual;
************/
std_msgs::Float32MultiArray PowerStatus[4];
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
ros::Publisher assessActual[4] = {handle.advertise<std_msgs::UInt16MultiArray>("WheelActual00", 2), \
                                  handle.advertise<std_msgs::UInt16MultiArray>("WheelActual01", 2), \
                                  handle.advertise<std_msgs::UInt16MultiArray>("WheelActual02", 2), \
                                  handle.advertise<std_msgs::UInt16MultiArray>("WheelActual03", 2)};

ros::Publisher assessPower[4] = {handle.advertise<std_msgs::Float32MultiArray>("UnitPower00", 2), \
                                 handle.advertise<std_msgs::Float32MultiArray>("UnitPower01", 2), \
                                 handle.advertise<std_msgs::Float32MultiArray>("UnitPower02", 2), \
                                 handle.advertise<std_msgs::Float32MultiArray>("UnitPower03", 2)};

unsigned long time_last;                 //for Buffer flushing
unsigned long time_last_query;           //for Query


void Actuate0( const std_msgs::UInt16MultiArray& ctrl_var){
    int i=0;
    //Send Signals to stepper motor and BLDC according to messages subscribed
    drive_input[i] = ctrl_var.data[1];
	if(ctrl_var.data[3]>0){ // 倒车
        vel[i] = vel[i] - drive_input[i];
	}//adjust for the switch
    else{
		vel[i] = vel[i] + drive_input[i];
	}
	if (ctrl_var.data[2]==0){
	    vel[i] = vel[i]*0.98;
	}
	else{											//Breaking
		vel[i] = vel[i]*(0.98-ctrl_var.data[2]*0.7/255);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget[i] = ctrl_var.data[0];
}

void Actuate1( const std_msgs::UInt16MultiArray& ctrl_var){
    int i=1;
    //Send Signals to stepper motor and BLDC according to messages subscribed
    drive_input[i] = ctrl_var.data[1];
	if(ctrl_var.data[3]>0){ // 倒车
        vel[i] = vel[i] - drive_input[i];
	}//adjust for the switch
    else{
		vel[i] = vel[i] + drive_input[i];
	}
	if (ctrl_var.data[2]==0){
	    vel[i] = vel[i]*0.98;
	}
	else{											//Breaking
		vel[i] = vel[i]*(0.98-ctrl_var.data[2]*0.7/255);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget[i] = ctrl_var.data[0];
}

void Actuate2( const std_msgs::UInt16MultiArray& ctrl_var){
    int i=2;
    //Send Signals to stepper motor and BLDC according to messages subscribed
    drive_input[i] = ctrl_var.data[1];
	if(ctrl_var.data[3]>0){ // 倒车
        vel[i] = vel[i] - drive_input[i];
	}//adjust for the switch
    else{
		vel[i] = vel[i] + drive_input[i];
	}
	if (ctrl_var.data[2]==0){
	    vel[i] = vel[i]*0.98;
	}
	else{											//Breaking
		vel[i] = vel[i]*(0.98-ctrl_var.data[2]*0.7/255);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget[i] = ctrl_var.data[0];
}

void Actuate3( const std_msgs::UInt16MultiArray& ctrl_var){
    int i=3;
    //Send Signals to stepper motor and BLDC according to messages subscribed
    drive_input[i] = ctrl_var.data[1];
	if(ctrl_var.data[3]>0){ // 倒车
        vel[i] = vel[i] - drive_input[i];
	}//adjust for the switch
    else{
		vel[i] = vel[i] + drive_input[i];
	}
	if (ctrl_var.data[2]==0){
	    vel[i] = vel[i]*0.98;
	}
	else{											//Breaking
		vel[i] = vel[i]*(0.98-ctrl_var.data[2]*0.7/255);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget[i] = ctrl_var.data[0];
}

//***MODIFY UNIT-SPECIFIC TOPICS AS NECESSARY!!!***//
ros::Subscriber sub[4] ={handle.subscribe("WheelControl00", 2, &Actuate0), \
                         handle.subscribe("WheelControl01", 2, &Actuate1), \
                         handle.subscribe("WheelControl02", 2, &Actuate2), \
                         handle.subscribe("WheelControl03", 2, &Actuate3)};

// the function to determine the wave pattern to servo
bool V_last = false;
void Flip(bool direc, uint8_t which_one) {
  if (direc == 0){
    angle_real[which_one] += 360/3000;
  }
  else if (direc == 1) {
    angle_real[which_one] -= 360/3000;
  }
}

void Steering(){
for (int i=0; i<4; i++){
	//-------------------start angle control------------------------------------
	if(steeringTarget[i] < 0 || steeringTarget[i] >encoder_resolution-1) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
		int err = (steeringTarget[i]-Angle[i]+uint16_t(0.5*encoder_resolution))%(encoder_resolution)-(encoder_resolution*0.5);
		//min(min(abs(steeringTarget-Angle),abs(steeringTarget-Angle+encoder_resolution)),abs(steeringTarget-Angle-encoder_resolution));
        if(!(abs(err)<40)) {
			pulseTime[i] = 7000/(1.1*abs(err)+5);	//Need Modification
            if (err<0) {
                Flip(0, i);
            }
            else {
                Flip(1, i);
            }  
        }
        //end while loop 
    }
    }
	//----------------end angle control---------------------------------------------  
}

void setup() {
	
	for (int i=0; i<4; i++){
    //PowerStatus.layout.dim_length = 1;
    //malloc(sizeof(std_msgs::MultiArrayDimension) * 3);
    PowerStatus[i].layout.dim.push_back(std_msgs::MultiArrayDimension());
    PowerStatus[i].layout.dim[0].label = "UnitPower";
    PowerStatus[i].layout.dim[0].size = 3;
    PowerStatus[i].layout.dim[0].stride = 1*3;
    //PowerStatus.layout.data_offset = 0;
    //PowerStatus[i].data = (float *)malloc(sizeof(float)*3);
    //PowerStatus[i].data_length = 3;
    for (int j = 0; j < 3; j++){
        PowerStatus[i].data.push_back(0.0);
        }
    
    //ActuatorStatus.layout.dim_length = 1;
    //malloc(sizeof(std_msgs::MultiArrayDimension) * 2);
    ActuatorStatus[i].layout.dim.push_back(std_msgs::MultiArrayDimension());
    ActuatorStatus[i].layout.dim[0].label = "WheelActual";
    ActuatorStatus[i].layout.dim[0].size = 2;
    ActuatorStatus[i].layout.dim[0].stride = 1*2;
    //ActuatorStatus.layout.data_offset = 0;
    //ActuatorStatus[i].data = (uint16_t *)malloc(sizeof(uint16_t)*2);
    //ActuatorStatus[i].data_length = 2;
    for (int j = 0; j < 2; j++){
        PowerStatus[i].data.push_back(0);
        }
    
    steeringTarget[i] = ActuatorStatus[i].data[0];	//Stop Init Steering of the wheel
    }
	
	//Initiate Steering Actuator and run via FlexiTimer
	//Query();
	//FlexiTimer2::set(updateTime,Query);  //time in the unit of ms
    //FlexiTimer2::start();
}

void loop() {
   unsigned long time_now = micros();
   if ((unsigned long)(time_now - time_last) > pulseTime){
       Steering();
       time_last = micros();
   }
   if ((unsigned long)(time_now - time_last_query) > updateTime){
       Query();
       time_last_query = micros();
   }
   handle.spinOnce();
}

void Query()
{
  uint16_t dataActuator[2] = {0};
  float dataPower[3];
  
  /*
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
  */

  dataActuator[0] = angle_real/360*4096+_zero;
  dataActuator[1] = dataActuator[1]+vel/4096;
  Angle=(dataActuator[0]-_zero+encoder_resolution)%encoder_resolution;
  Speed=(-dataActuator[1]+_last+encoder_resolution)%encoder_resolution;
  _last = dataActuator[1];
  //Angle=dataActuator[0];
  
  dataPower[0] = 50*(0.02892+0.00002576*50)+2.99;
  dataPower[1] = 5.0;
  dataPower[2] = drive_input[0]*vel[0]/62500;
  
  ActuatorStatus.data = dataActuator;
  PowerStatus.data = dataPower;
  
  assessActual.publish (&ActuatorStatus);
  assessPower.publish (&PowerStatus);
  //Serial.write((const uint8_t*)&to_send,sizeof(serial_format));
}

void main(){
    setup();
    while (ros::ok()){
        loop();
    }
}


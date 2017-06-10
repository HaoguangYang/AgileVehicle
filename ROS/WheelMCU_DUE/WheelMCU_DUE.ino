#include "ros_customized.h"
//#include <PWM.h>
//#include "pwm_lib.h"
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Int32MultiArray.h>

//#define CONTRL      3 //PWM
//#define BACK        4
//#define BREAK       5 //connecting to driving motor controller, PWM
#define csn        10 //yellow
#define datS       11 //green
#define datD       12 //green
#define clk        14 //blue
#define ENA        13
#define PUL        15
#define DIR        16 // direction for the angle motor
#define VOLT        A3
#define AMPD        A7
#define AMPS        A6

ros::NodeHandle handle;
const uint16_t encoder_resolution = 4096;
const uint16_t         updateTime = 40000;	//Time in us
const uint16_t          queryTime = 1000;	//Time in us
uint16_t                pulseTime = 70;    //Time in us
uint16_t                    Angle = 0;
uint16_t           steeringTarget = 0;
const uint16_t _zero=0; 
float throttle=1.0;
uint8_t perf_flag=0;                        //For Performance Meter

//BLDC TEST REGION
int32_t frequency = 20000; //pwm frequency in Hz
unsigned int n = 0, timer2_initial_value, s = 0;
bool BACK;
uint32_t pwmPin =9;
uint32_t maxDutyCount = 256;
uint32_t clkAFreq = 42000000ul;
uint32_t pwmFreq = 42000000ul; 
uint32_t channel;

//uint16_t dataActuator[2];
//float dataPower[3];

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
ros::Publisher assessActual("WheelActual00", &ActuatorStatus);
ros::Publisher assessPower("UnitPower00", &PowerStatus);

unsigned long time_last;                 //for Buffer flushing
unsigned long time_last_query;           //for Query
unsigned long time_last_publish;		 //for Publish

void Actuate( const std_msgs::UInt16MultiArray& ctrl_var){
    //Send Signals to stepper motor and BLDC according to messages subscribed
	if(ctrl_var.data[3]>0){ // 倒车
        BACK = true;
        //digitalWrite(BACK,HIGH);
	}//adjust for the switch
    else{
        BACK = false;
		//digitalWrite(BACK,LOW);
	}
	uint16_t drive_input = ctrl_var.data[1];
	if (ctrl_var.data[2]==0){
		//ctrl_var.data[1] = ctrl_var.data[1] + 75;	//Calibration of controller to elliminate dead zone
		//analogWrite(BREAK,0);
		//analogWrite(CONTRL,drive_input*throttle);	//Normal Driving
		PWMC_SetDutyCycle(PWM_INTERFACE, channel, drive_input*throttle/4);
		//pwmWrite(9,drive_input*throttle/4);
	}
	else{											//Breaking
		//analogWrite(CONTRL,0);
		//analogWrite(BREAK,ctrl_var.data[2]);
		PWMC_SetDutyCycle(PWM_INTERFACE, channel, 0);
		//pwmWrite(9,0);
	}
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
   steeringTarget = encoder_resolution-1-ctrl_var.data[0];
}

//***MODIFY UNIT-SPECIFIC TOPICS AS NECESSARY!!!***//
ros::Subscriber<std_msgs::UInt16MultiArray> sub("WheelControl00", &Actuate);

void Steering(){
	//-------------------start angle control------------------------------------
	if(steeringTarget > encoder_resolution-1) { // will be modified as -90 degree to 90 degree
        //Serial.println("bad DesiredAngle input.");
    }
    else {
		int16_t err = (steeringTarget-Angle+(uint16_t)(0.5*encoder_resolution))%(encoder_resolution)-(encoder_resolution*0.5);
		//min(min(abs(steeringTarget-Angle),abs(steeringTarget-Angle+encoder_resolution)),abs(steeringTarget-Angle-encoder_resolution));
        if(!(abs(err)<40)) {
            digitalWrite(ENA, LOW);
			pulseTime = 7000/(1.1*abs(err)+5);	//Need Modification
            if (err<0) {
                Flip(0);
            }
            else {
                Flip(1);
            }  
        }
        else digitalWrite(ENA, HIGH);
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
	
	//BLDC TEST REGION
	//InitTimersSafe();
    //bool success = SetPinFrequencySafe(9, frequency);
    pinMode(3, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);
    pinMode(8, OUTPUT);
    pinMode(13, OUTPUT);
    pinMode(0, INPUT);
    pinMode(1, INPUT);
    pinMode(2, INPUT);
    //pinMode(10, INPUT_PULLUP);
    //pinMode(11, INPUT_PULLUP);
    //pinMode(12, INPUT_PULLUP);
    // initialize timer2 interrupt for adc reading 
    noInterrupts();           // disable all interrupts
    //TCCR2A = 0;
    //TCCR2B = 0;
    timer2_initial_value = 0;  
    //TCNT2 = timer2_initial_value;   // preload timer
    //TCCR2B |= (1 << CS22) |(1 << CS21) | (1 << CS20); // 1024 prescaler 
    //TIMSK2 |= (1 << TOIE2);   // enable timer overflow interrupt
    //interrupts();             // enable all interrupts
    
    //digitalWrite(13, LOW);
    
    digitalWrite(8,0);digitalWrite(7,0);digitalWrite(6,0);
    digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,0);
    pmc_enable_periph_clk(PWM_INTERFACE_ID);
  PWMC_ConfigureClocks(clkAFreq, 0, VARIANT_MCK);
 
  PIO_Configure(
    g_APinDescription[pwmPin].pPort,
    g_APinDescription[pwmPin].ulPinType,
    g_APinDescription[pwmPin].ulPin,
    g_APinDescription[pwmPin].ulPinConfiguration);
 
  channel = g_APinDescription[pwmPin].ulPWMChannel;
  PWMC_ConfigureChannel(PWM_INTERFACE, channel , pwmFreq, 0, 0);
  PWMC_SetPeriod(PWM_INTERFACE, channel, maxDutyCount);
  PWMC_EnableChannel(PWM_INTERFACE, channel);
  PWMC_SetDutyCycle(PWM_INTERFACE, channel, s);
 
  pmc_mck_set_prescaler(2);
  //pwmWrite(9,s);
}

/*ISR(TIMER2_OVF_vect)        // interrupt service routine 
{
  TCNT2 = timer2_initial_value;   // preload timer
  n++;
  if (n>20){
    n = 0;
   if (s != analogRead(A0)){
   s = analogRead(A4); 
  pwmWrite(9,s/4);}}
}*/

int fwd(){
   
if (digitalRead(2)==1){
    if (digitalRead(1)==0){
      if (digitalRead(0)==1){
        digitalWrite(8,0);digitalWrite(7,0);digitalWrite(6,0);
        digitalWrite(5,1);digitalWrite(4,1);digitalWrite(3,0);}
      else {
        digitalWrite(8,1);digitalWrite(7,0);digitalWrite(6,0);
        digitalWrite(5,1);digitalWrite(4,0);digitalWrite(3,0);}}
        
     if (digitalRead(1)==1){
      if (digitalRead(0)==0){
        digitalWrite(8,1);digitalWrite(7,0);digitalWrite(6,0);
        digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,1);}}}
        
   
 if (digitalRead(2)==0){
    if (digitalRead(1)==1){
      if (digitalRead(0)==0){
        digitalWrite(8,0);digitalWrite(7,0);digitalWrite(6,1);
        digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,1);}
      else {
        digitalWrite(8,0);digitalWrite(7,1);digitalWrite(6,1);
        digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,0);}}
        
     if (digitalRead(1)==0){
      if (digitalRead(0)==1){
        digitalWrite(8,0);digitalWrite(7,1);digitalWrite(6,0);
        digitalWrite(5,0);digitalWrite(4,1);digitalWrite(3,0);}}}
}

int bwd(){
if (digitalRead(2)==1){
    if (digitalRead(1)==0){
      if (digitalRead(0)==1){
        digitalWrite(8,0);digitalWrite(7,0);digitalWrite(6,1);
        digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,1);}
      else {
        digitalWrite(8,0);digitalWrite(7,1);digitalWrite(6,1);
        digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,0);}}
        
     if (digitalRead(1)==1){
      if (digitalRead(0)==0){
        digitalWrite(8,0);digitalWrite(7,1);digitalWrite(6,0);
        digitalWrite(5,0);digitalWrite(4,1);digitalWrite(3,0);}}}
        
   
 if (digitalRead(2)==0){
    if (digitalRead(1)==1){
      if (digitalRead(0)==0){
        digitalWrite(8,0);digitalWrite(7,0);digitalWrite(6,0);
        digitalWrite(5,1);digitalWrite(4,1);digitalWrite(3,0);}
      else {
        digitalWrite(8,1);digitalWrite(7,0);digitalWrite(6,0);
        digitalWrite(5,1);digitalWrite(4,0);digitalWrite(3,0);}}
        
     if (digitalRead(1)==0){
      if (digitalRead(0)==1){
        digitalWrite(8,1);digitalWrite(7,0);digitalWrite(6,0);
        digitalWrite(5,0);digitalWrite(4,0);digitalWrite(3,1);}}}
}

void loop() {
   unsigned long time_now = micros();
   if ((uint32_t)(time_now - time_last) > pulseTime){
       Steering();
       time_last = micros();
   }
   if ((uint32_t)(time_now - time_last_query) > queryTime){
       Query();
       Throttling();
       time_last_query = micros();
       perf_flag = 0;
   }
   if ((uint32_t)(time_now - time_last_publish) > updateTime){
	   Publish();
	   time_last_publish = micros();
   }
   if (BACK == true) bwd();
   else fwd();
   perf_flag++;
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
  ActuatorStatus.data[0] = 0;
  ActuatorStatus.data[1] = 0;
  digitalWrite(csn,LOW);
  delayMicroseconds(1);
  for (uint8_t k=0;k<12;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
    ActuatorStatus.data[0]=(ActuatorStatus.data[0]<<1)+digitalRead(datS);		//Steering
    ActuatorStatus.data[1]=(ActuatorStatus.data[1]<<1)+digitalRead(datD);		//Driving
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
  Angle=(ActuatorStatus.data[0]-_zero+encoder_resolution)%encoder_resolution;
  ActuatorStatus.data[0] = Angle;
  //Speed=(-dataActuator[1]+_last+encoder_resolution)%encoder_resolution;
  //_last = dataActuator[1];
  //Angle=dataActuator[0];
  ActuatorStatus.data[1] = perf_flag;
  PowerStatus.data[0] = analogRead (VOLT)*(0.02892+0.00002576*analogRead (VOLT))+2.99;
  PowerStatus.data[1] = (analogRead (AMPS)-512)*30/409.6;
  PowerStatus.data[2] = (analogRead (AMPD)-512)*30/409.6;
}

void Publish(){
    //ActuatorStatus.data = dataActuator;
    //PowerStatus.data = dataPower;
    assessActual.publish (&ActuatorStatus);
    assessPower.publish (&PowerStatus);
}


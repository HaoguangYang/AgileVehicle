#include <FlexiTimer2.h>
#define CONTRL      3
#define INT         2
#define BACK        4
#difine BREAK		5
#define csn        13 //yellow
#define dat        11 //green
#define clk         9 //blue
#define PUL         7
#define DIR         6 // direction for the angle motor
// Drive
unsigned long tep_time = 0;
int count = 0;
void Motorspeed(void);
int DutyCycle=0;
int Freq=0;
// Steering
unsigned short data=0;
unsigned short Angle=0;
unsigned short val=0;
int PulseNum;
int driver_resolution=6400;
int encoder_resolution=4096;
int DesiredAngle=encoder_resolution/2;// initial the first desiredangle "0degree"
int reduction_ratio=15;
int AngleTime=200; //in ms
int PulseTime=100; //in ms, the cycle for angle motor
//data input
String inputString ="";
String inputDutycycle="";
String inputAngle="";
String inputBreak="";
boolean SpeedComplete = false;
boolean AngleComplete = false;
boolean BreakInit = false;

typedef struct {
  char end_1;
  char end_2;
  unsigned short x;
  unsigned short y;
} serial_format;

void setup() {
  // set the COM:
  Serial.begin(9600);
  inputString.reserve(100);
  inputDutycycle.reserve(100);
  inputAngle.reserve(100);
 // set driving control
  pinMode(CONTRL, OUTPUT);
  pinMode(INT,    INPUT);
  attachInterrupt(0, Motorspeed, RISING);
  analogWrite(CONTRL, 1);
 // set steering
  pinMode(csn,OUTPUT);
  pinMode(dat,INPUT);
  pinMode(clk,OUTPUT);
  pinMode(DIR,OUTPUT);
  pinMode(PUL,OUTPUT);
  digitalWrite(csn,HIGH);
  digitalWrite(clk,HIGH);
  encoder();
  FlexiTimer2::set(AngleTime,encoder);  //time in the unit of ms
  FlexiTimer2::start();
}
void Motorspeed()
{
  if(count == 24){
    count = 0;
    Freq=1.0 / (((double)(micros() - tep_time ) / 1000000.0)/2.0);
    //Serial.println(Freq);
    tep_time = micros();
  }
  else {
    count++;
  }
}
void encoder()
{
  data=0;
  digitalWrite(csn,LOW);
  delayMicroseconds(1);
  for (int k=0;k<12;k++)
  {
    digitalWrite(clk,LOW);
    delayMicroseconds(1);
    digitalWrite(clk,HIGH);
    delayMicroseconds(1);
    val=digitalRead(dat);
    data=(data<<1)+val;
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
  
  serial_format to_send;
  to_send.end_1=0x3f;
  to_send.end_2=0x3f;
  to_send.x=data;
  to_send.y=Freq;
  Serial.write((const uint8_t*)&to_send,sizeof(serial_format));
  //Serial.print("Angle:");
  //Serial.println(data);
  //Serial.print("Frequency:");
  //Serial.println(Freq);
  //Angle=data;
}

// the function to move the angle motor for once
void OneUp(unsigned int time, unsigned int direc) {
  if (direc == 1){
    digitalWrite(DIR,HIGH);
  }
  else if (direc == 2) {
    digitalWrite(DIR,LOW);
  }
  // give a pulse
  digitalWrite(PUL,HIGH);
  delayMicroseconds(time);
  digitalWrite(PUL,LOW);
  delayMicroseconds(time);
   //unit of time is us
}

void loop() {
    // Speed------------------------------------------------------------------
    if(SpeedComplete)
    { 
        DutyCycle = inputDutycycle.toInt(); // 从string转化为数字
        analogWrite(CONTRL,DutyCycle);
        SpeedComplete = false;
        //Serial.println(DutyCycle);
        inputDutycycle="";
    }
	// Breaking---------------------------------------------------------------
	if (BreakInit)
	{
		digitalWrite(BREAK,HIGH);
		analogWrite(CONTRL,0);
		//ADD BREAKING HYDRAULICS ENGAGE CODE HERE.
		BreakInit = false;
	}
    // Angle-------------------------------------------------------------------
    if(AngleComplete)
    {
        DesiredAngle = inputAngle.toInt();
        AngleComplete = false;
        //Serial.print("DesireAngle:");
        //Serial.println(DesiredAngle);
        inputAngle="";
    }
    // As the first version has only one encoder (for the angle), only the angle part has the close loop control.
    //-------------------start angle control------------------------------------
    if(!(DesiredAngle < 0 || DesiredAngle >encoder_resolution)) {
        if(!(abs(DesiredAngle-Angle)<40 ||abs(DesiredAngle-Angle+encoder_resolution)<40||abs(DesiredAngle-Angle-encoder_resolution)<40)) {
            if (DesiredAngle>Angle) {
                OneUp(PulseTime,1);
            }
            else {
                OneUp(PulseTime,2);
            }  
        }
    }
	//ADD BREAKING HYDRAULICS ENGAGE CODE HERE.
    //----------------end angle control---------------------------------------------  
    //end loop 
}

void serialEvent(){
    while(Serial.available()){
        char inChar = (char)Serial.read();
        if((inChar != 's') && (inChar != 'd') && (inChar != 'b') && (inChar != 'r') && (inChar != 'f')){
            inputString+=inChar;
        }
        if(inChar == 's'){ // s是转角数据开头
            AngleComplete = true;
            inputAngle=inputString;
            inputString="";
            return;
        }
        if(inChar == 'd'){ // d是驱动数据开头
            SpeedComplete = true;
            inputDutycycle=inputString;
            inputString="";
            //Serial.println("aaa");
            return;
        }
		if(inChar == 'b'){ // b是刹车数据开头
            BreakInit = true;
			inputBreak = inputString;
            inputString="";
            return;
        }
        if(inChar == 'r'){ // r是倒车开头
            digitalWrite(BACK,HIGH); //adjust for the switch
            return;
        }
        if(inChar == 'f'){ // 取消倒车
            digitalWrite(BACK,LOW); //adjust for the switch
            return;
        }
    }
    //END WHILE LOOP
}


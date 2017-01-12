// data example for speed control: S93
// data example for steering angle control: A93

String comdata = "";
int mk = 0;
//int mk_ctrl[2] = {0};
int numdata[2]={93,93};
int numlast[2]={93,93};
int pwmpin[2]={11,12}; //pin 11 for steering angle, pin 12 for speed

void setup()
{
  for(int i=0;i<2;i++)
  {
    pinMode(pwmpin[i],OUTPUT);
  }
  TCCR1A = B10101011; //set 10-bit fast PWM
  TCCR1B = TCCR1B & B11100000 | B00001100; // set timer 1 divisor to  1024 for PWM frequency of 61.04 Hz
  //motor: 97mid 61min 124max
  //steer: 
  Serial.begin(9600);
  analogWrite(11,numlast[0]);
  analogWrite(12,numlast[1]);
  Serial.print("Initial Steering Angle = ");
  Serial.println(numlast[0]);
  Serial.print("Initial Speed = ");
  Serial.println(numlast[1]);
}

void loop()
{
    int len[2]={(numdata[0]-numlast[0]),(numdata[1]-numlast[1])};
    for(int j=0;j<2;j++)
    {
      for(int i=0;i<abs(len[j]);i++)
      {
        analogWrite(pwmpin[j],numlast[j]+(i+1)*len[j]/abs(len[j]));
        delay(100);
      }
    }
    for(int i=0;i<2;i++)
    {
      numlast[i]=numdata[i];
    }
}

void serialEvent()
{
	mk = 0;
	while (Serial.available())
	{
		char inChar = (char)Serial.read();
		if (inChar == '$'){ comdata = ""; }	//clear buffer
		if ((inChar != 'v') && (inChar != 'a') && (inChar != '$'))
		{
			comdata += inChar;				//input buffer
		}
		if (inChar == 'a')
		{
			mk = true;
			numdata[0] = comdata.toInt();	//angle
			comdata = "";
			Serial.print("Steering Angle = ");
			Serial.println(numdata[0]);
			Serial.print("Speed = ");
			Serial.println(numdata[1]);
			return;
		}
		if (inChar == 'v'){
			mk = true;
			numdata[1] = comdata.toInt();	//speed
			comdata = "";
			//Serial.println("aaa");
			Serial.print("Steering Angle = ");
			Serial.println(numdata[0]);
			Serial.print("Speed = ");
			Serial.println(numdata[1]);
			return;
		}
	}
}

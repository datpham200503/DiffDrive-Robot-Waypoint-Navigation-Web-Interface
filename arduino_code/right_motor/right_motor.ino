#include <PinChangeInt.h>
#include <PID_v1.h>
#define encodPinA1      2
#define encodPinB1      4
#define M1              5
#define M2              6

double kp =1, ki =20 , kd =0;
double input = 0, output = 0, setpoint = 0;
unsigned long lastTime,now;
volatile long encoderPos = 0,last_pos=0,lastpos=0;
PID myPID(&input, &output, &setpoint, kp, ki, kd,DIRECT);  
void setup() {
  pinMode(encodPinA1, INPUT_PULLUP);
  pinMode(encodPinB1, INPUT_PULLUP);
  attachInterrupt(0, encoder, FALLING);
  TCCR1B = TCCR1B & 0b11111000 | 1;
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(1);
  myPID.SetOutputLimits(-255, 255);
  
  Serial.begin (9600); 
}

void loop() {
   now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=500 )
   {
      input = (360.0*1000*(encoderPos-last_pos)) /(1856.0*(now - lastTime));
      lastTime=now;
      last_pos=encoderPos;
   }
  
  myPID.Compute();                                    // calculate new output
  pwmOut(output);                                     // drive L298N H-Bridge module
  delay(10);
}

void pwmOut(int out) {                                // to H-Bridge board
  if (out > 0) {
    analogWrite(M1, 0);                             // drive motor CW
    analogWrite(M2, out);
  }
  else {
    analogWrite(M1, abs(out));
    analogWrite(M2, 0);                        // drive motor CCW
  }
}

void encoder()  {                                     // pulse and direction, direct port reading to save cycles  
  if (PINB & 0b00000001)    encoderPos++;             // if(digitalRead(encodPinB1)==HIGH)   count ++;
  else                      encoderPos--;             // if(digitalRead(encodPinB1)==LOW)   count --;
}

void requestEvent() {
  int8_t s;
  
  s= (360.0*(encoderPos-lastpos))/1856.0; //change in position in degrees of the wheel
  lastpos=encoderPos;
}


// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  uint8_t a,b;
  setpoint= (double)((b<<8)|a);
  Serial.println((uint8_t)a);
  Serial.println((uint8_t)b);
  
  Serial.println(setpoint);
}

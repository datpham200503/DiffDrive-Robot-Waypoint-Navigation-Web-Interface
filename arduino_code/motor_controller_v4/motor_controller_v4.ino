/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */
#include <TimerOne.h>   // thu vien ngat timer 1
#include <ros.h>  // thu vien ros
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include "HardwareSerial.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include "Simple_MPU6050.h"
#define MPU6050_DEFAULT_ADDRESS 0x68  // address pin low (GND)
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  // Default
Simple_MPU6050 mpu(Three_Axis_Quaternions);
#define OFFSETS -3466, 5646, 8494, 20, -69, -19

//pid//
float KP[] = { 30, 30 };  // Thô?ng s?? Kp b?? ??i??u khi??n PID: kp[0] la banh trai kp[1] la banh phai
float KI[] = { 0.2, 0.2};  // Thô?ng s?? Ki b?? ??i??u khi??n PID: kp[0] la banh trai kp[1] la banh phai
float KD[] = { 0.0, 0.0 };    // Thô?ng s?? Kd b?? ??i??u khi??n PID: kp[0] la banh trai kp[1] la banh phai
double deltaT = 0.01;         // th??i gian l??y m??u
//thong so xe//
double L = 0.138;               // kho??ng cá?ch giua 2 bá?nh xe
double R = 0.032;            // bá?n kí?nh bá?nh xe
    // Distance between the wheels
//Motor control Pin//
int leftpwm = 9; //Control pwm left motor
int leftdir = 6;  //Control direction left motor
int leftenb = 5;

int leftencoder_a = 18;  // Read state encoder channel LOW or HIGH
int leftencoder_b = 4;

int righpwm = 10; //Control pwm right motor
int righdir = 7; //Control direction right motor;
int righenb = 8;

int righencoder_a = 3;
int righencoder_b = 11;

//Variable dem xung va tinh toan quang duong
int pulLeft=0;
int pulRight=0;
int pulPreRight = 0, pulPreLeft = 0;
int delta_tick_right = 0;
int delta_tick_left = 0;
float distanceRight = 0, distanceLeft = 0;
// Variables for robot position tracking
float delta_s = 0;  // Total distance traveled from the center
float x = 0;        // x distance
float y = 0;        // y distance
float theta = 0;    // Angular position
float delta_theta = 0;
float last_theta = 0;
float theta_temp = 0;

//Variable error to calc u PID
int pwmValRight = 0, pwmValLeft = 0;  // pwm cho ????ng c??
float speedLeft = 0, speedRight = 0;
float velLinear = 0, angLinear = 0;
float velLeft = 0, velRight = 0;
float desiredVelRight = 0, desiredVelLeft = 0;
float errVelRight = 0, errVelLeft = 0;
float errTotalRight = 0, errTotalLeft = 0;
float errPrevRight = 0, errPrevLeft = 0;
float errDiffRight = 0, errDiffLeft = 0;

// Time interval for measurements in milliseconds
const int interval_pub = 400;
const int interval_nh = 2;
long currentMillis = 0;
long previousMillis_pub = 0;
long previousMillis_nh = 0;
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
#define PI 3.1415926535897932384626433832795

// ----- ROS NodeHandle ----- //
ros::NodeHandle nh;

// ------------Hà?m ??i??u khi??n ????ng c?? - DUC----------------
void controlMotor(int valPWM, int PWM, int dir_D, int enb_D) {
  if (valPWM > 0 ){
    digitalWrite(enb_D, 0); 
    digitalWrite(dir_D, 1);  
  }
  else if (valPWM < 0 ){
    digitalWrite(enb_D, 1);
    digitalWrite(dir_D, 0);    
  }
  else{
    digitalWrite(enb_D, 0);
    digitalWrite(dir_D, 0);
  }
  // xuat xung PWM
  analogWrite(PWM, abs(valPWM));
}

//----------------Ham tinh toan van toc 2 banh--------------------//
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg) {
  velLinear = (float)cmd_vel_msg.linear.x;
  angLinear = (float)cmd_vel_msg.angular.z;
  velLinear = constrain(velLinear, -(R* 2 * PI * 30 / 60), R* 2 * PI * 30 / 60);
  angLinear = constrain(angLinear, -(R* 2 * PI * 30 / 60)/0.069 ,(R* 2 * PI * 30 / 60)/0.069);
  float omegaL = (2*velLinear - angLinear*L) / (2*R);         // rad/s
  float omegaR = (2*velLinear + angLinear*L) / (2*R);       // rad/s
  desiredVelLeft = omegaL * (30/PI);    // rpm
  desiredVelRight = omegaR * (30/PI);  // rpm
  desiredVelLeft = constrain(desiredVelLeft, -100, 100);
  desiredVelRight = constrain(desiredVelRight, -100, 100);
}
// ----- Subscriber ----- //
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &commandVelocityCallback);

//--------- Hà?m ????c xung t?? encoder bá?nh trá?i-----------
void readEncoderLeft() {
  if (digitalRead(leftencoder_b) == digitalRead(leftencoder_a)) pulLeft--;
  else pulLeft++;
}

// --------Hà?m ????c xung t?? encoder bá?nh ph??i---------
void readEncoderRight() {
  if (digitalRead(righencoder_b) == digitalRead(righencoder_a)) pulRight++;
  else pulRight--;
}

//------PID-----//
void funcVelPID() {
  velLeft = ((float)delta_tick_left / 2200) * (1 / deltaT) * 60;    // rpm
  velRight = ((float)delta_tick_right / 2200) * (1 / deltaT) * 60;  // rpm
  delta_tick_right = pulRight - pulPreRight;
  delta_tick_left = pulLeft - pulPreLeft;
  pulPreRight = pulRight;
  pulPreLeft = pulLeft;
  // Sai s?? gi??a v??n t??c ????t và? v??n t??c hi??n t??i
  errVelLeft = desiredVelLeft - velLeft;
  errVelRight = desiredVelRight - velRight;

  // tí?ch phâ?n d??ng ????n gi??n
  errTotalLeft += errVelLeft * deltaT;
  errTotalRight += errVelRight * deltaT;

  // vi phâ?n d??ng ????n gi??n
  errDiffLeft = (errVelLeft - errPrevLeft) / deltaT;
  errDiffRight = (errVelRight - errPrevRight) / deltaT;
  errPrevLeft = errVelLeft;
  errPrevRight = errVelRight;
  
  // b?? ??i??u khi??n PID
  pwmValLeft = errVelLeft * KP[0] + errTotalLeft * KI[0] + errDiffLeft * KD[0];
  pwmValRight = errVelRight * KP[1] + errTotalRight * KI[1] + errDiffRight * KD[1];

  pwmValLeft = constrain(pwmValLeft, -255, 255);
  pwmValRight = constrain(pwmValRight, -255, 255);
}

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(leftpwm, OUTPUT);
  pinMode(leftdir, OUTPUT);
  pinMode(leftenb, OUTPUT);
  
  pinMode(righpwm, OUTPUT);
  pinMode(righdir, OUTPUT);
  
  pinMode(righenb, OUTPUT);
  
  TCCR2B = TCCR2B & B11111000 | B00000001;
  
  //Input pin read encoder//
  pinMode(leftencoder_a, INPUT_PULLUP);
  pinMode(leftencoder_b, INPUT);
  
  pinMode(righencoder_a, INPUT_PULLUP);
  pinMode(righencoder_b, INPUT);

  //interrupt encoder//
  attachInterrupt(digitalPinToInterrupt(leftencoder_a), readEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(righencoder_a), readEncoderRight, RISING);

  //ros setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(subCmdVel); 

  Timer1.initialize(10000);  // 10000uS --> 0.01s
  Timer1.attachInterrupt(funcVelPID);
}

void loop() {
  currentMillis = millis();  // Record the time
  if (currentMillis - previousMillis_nh > interval_nh) {
    controlMotor(pwmValLeft, leftpwm, leftdir, leftenb);
    controlMotor(pwmValRight, righpwm, righdir, righenb); 
    nh.spinOnce();
    previousMillis_nh = currentMillis;
  }

}

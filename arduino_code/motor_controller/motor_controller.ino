#include "I2Cdev.h"
#include "MPU9250_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif
#include <ros.h>
#include <PID_v1.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
MPU9250 mpu;
#define OUTPUT_READABLE_QUATERNION
#define INTERRUPT_PIN 12 // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
ros::NodeHandle nh;
geometry_msgs::Quaternion quat_msg;
ros::Publisher quat_pub("quaternion", &quat_msg);

// Handles startup and shutdown of ROS
ros::NodeHandle nh;

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
 mpuInterrupt = true;
}

////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 11

//--- Speed-specific constants ---
double angular_speed = 0;
double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 
                        
const double max_speed = 0.1;                 //Max speed in m/s

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);
 
// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections
const int enA = 9;
const int in1 = 6;
const int in2 = 5;
  
// Motor B connections
const int enB = 10; 
const int in3 = 7;
const int in4 = 8;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 2150;
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.032;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.138;
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 10700; // Originally 2880
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K = 2550;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 50;
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 50; // about 0.02 m/s
const int PWM_MAX = 255; // about 0.1 m/s
 
// Set linear velocity and PWM variable values for each wheel
double pwmLeftReq = 0;
double pwmRightReq = 0;
 
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

// PID Parameters
const double PID_left_param[] = { 0.95, 10, 0 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0.95, 10, 0 }; //Respectively Kp, Ki and Kd for right motor PID

PID PID_leftMotor(&speed_act_left, &speed_cmd_left, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &speed_cmd_right, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor

/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 
void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  speed_act_left = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = (millis()/1000);
 
}
 
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /right_ticks topic. 
void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  speed_act_right = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  prevRightCount = right_wheel_tick_count.data;
   
  prevTime = (millis()/1000);
 
}
 
// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
  
  angular_speed = cmdVel.angular.z;
  speed_req_left = cmdVel.linear.x;
  speed_req_right = cmdVel.linear.x;
    
  // Check if we need to turn 
  if (angular_speed != 0.0) {
 
    // Turn left
    if (angular_speed > 0.0) {
      pwmLeftReq = -PWM_TURN;
      pwmRightReq = PWM_TURN;
    }
    // Turn right    
    else {
      pwmLeftReq = PWM_TURN;
      pwmRightReq = -PWM_TURN;
    }
  }
  // Go straight
}
 
void set_pwm_values() {
 
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  static bool stopped = false;
  if ((pwmLeftReq * speed_act_left < 0 && pwmLeftOut != 0) ||
      (pwmRightReq * speed_act_right < 0 && pwmRightOut != 0)) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  // Set the direction of the motors
  if (pwmLeftReq > 0) { // Left wheel forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else if (pwmLeftReq < 0) { // Left wheel reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }
  else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  else { // Left wheel stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
  }
 
  if (pwmRightReq > 0) { // Right wheel forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else if(pwmRightReq < 0) { // Right wheel reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }
  else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }
  else { // Right wheel stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW); 
  }
 
  // Increase the required PWM if the robot is not moving
  if (pwmLeftReq != 0 && speed_act_left == 0) {
    pwmLeftReq *= 1.5;
  }
  if (pwmRightReq != 0 && speed_act_right == 0) {
    pwmRightReq *= 1.5;
  }
 
  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += 1;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= 1;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += 1;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= 1;
  }
  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
 
  // Set the PWM value on the pins
  analogWrite(enA, pwmLeftOut); 
  analogWrite(enB, pwmRightOut); 
}
 
// Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );
 
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  Serial.begin(57600);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(228);
  mpu.setYGyroOffset(93);
  mpu.setZGyroOffset(-64);
  mpu.setZAccelOffset(1421); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
  // Calibration Time: generate offsets and calibrate our MPU9250
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.PrintActiveOffsets();
  // turn on the DMP, now that it's ready
  //Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  digitalPinToInterrupt(INTERRUPT_PIN);
  
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  // set our DMP Ready flag so the main loop() function knows it's okay to use it
  
  dmpReady = true;
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
  
  Serial.print(F("DMP Initialization failed (code "));
  Serial.print(devStatus);
  Serial.println(F(")"));
  }
  pinMode(LED_PIN, OUTPUT);
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
   
  // Motor control pins are outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set the motor speed
  analogWrite(enA, 0); 
  analogWrite(enB, 0);
  
  //setting PID parameters
  PID_leftMotor.SetSampleTime(95);
  PID_rightMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);
  PID_rightMotor.SetMode(AUTOMATIC);
  
  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(quat_pub);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
}

// Thong so thoi gian dung cho IMU
long previousMillisne = 0;
long currentMillisne = 0;
float deltane=0;

void loop() {
   
  nh.spinOnce();
  
  currentMillisne = micros();
  deltane=currentMillisne-previousMillisne;
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  { // Get the Latest packet
  #ifdef OUTPUT_READABLE_QUATERNION
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  quat_msg.w = q.w;
  quat_msg.x = q.x;
  quat_msg.y = q.y;
  quat_msg.z = q.z;
  
  quat_pub.publish(&quat_msg);
  
  #endif
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  }
  
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
 
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();

    // If robot go forward
    if(angular_speed == 0)
    {
      PID_leftMotor.Compute();
      PID_rightMotor.Compute();

      pwmLeftReq = constrain(K * speed_req_left, -255, 255);   
      pwmRightReq = constrain(K * speed_req_right, -255, 255);

      // Handle low PWM values
      if (abs(pwmLeftReq) < PWM_MIN) {
        pwmLeftReq = 0;
      }
      if (abs(pwmRightReq) < PWM_MIN) {
        pwmRightReq = 0;  
      } 
    }
        
  }
   
  // Stop the car if there are no cmd_vel messages
  if((millis()/1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }
 
  set_pwm_values();
  previousMillisne=currentMillisne;
}

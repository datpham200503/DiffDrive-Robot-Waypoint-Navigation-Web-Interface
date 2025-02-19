#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String OurLedState;
ros::Publisher LEDstate("state", &OurLedState);

char RedState[20] = "The Red LED blinks!";
char BlueState[21] = "The Blue LED Blinks!";

void RedOne( const std_msgs::Empty& toggle_msg) 
{  
digitalWrite(13, HIGH);
delay(3000); 
digitalWrite(13, LOW);
OurLedState.data = RedState;
} 

void BlueOne( const std_msgs::Empty& toggle_msg) 
{  
digitalWrite(12, HIGH);
delay(3000); 
digitalWrite(12, LOW);
OurLedState.data = BlueState;
} 

ros::Subscriber<std_msgs::Empty> RedLED("red", &RedOne ); 
ros::Subscriber<std_msgs::Empty> BlueLED("blue", &BlueOne ); 

void setup()  
{ 
pinMode(13, OUTPUT);
pinMode(12, OUTPUT); 
nh.initNode(); 
nh.subscribe(RedLED);
nh.subscribe(BlueLED);
nh.advertise(LEDstate);
} 

void loop()  
{  
LEDstate.publish( &OurLedState );
nh.spinOnce();  
delay(1);  
}

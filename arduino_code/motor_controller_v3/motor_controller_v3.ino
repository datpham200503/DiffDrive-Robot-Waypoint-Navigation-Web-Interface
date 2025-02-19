const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 4;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 11;              //B channel for encoder of right motor 

int pos_left = 0;       //Left motor encoder position
int pos_right = 0;      //Right motor encoder position


//__________________________________________________________________________

void setup() {
  Serial.begin(115200);
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT_PULLUP); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  attachInterrupt(digitalPinToInterrupt(2), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT_PULLUP); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  attachInterrupt(digitalPinToInterrupt(3), encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  Serial.println(pos_left);
 }

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_right--;
  else pos_right++;
}

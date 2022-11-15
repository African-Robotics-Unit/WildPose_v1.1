#include <Motoron.h>

#define HWSERIAL Serial3

// Motor driver settings
MotoronI2C mc(16);  // Motoron controller
#define WIRE Wire // using I2C 0
#define ENCODER_1A 14
#define ENCODER_1B 15
#define ENCODER_2A 11
#define ENCODER_2B 10
#define ENCODER_3A 5
#define ENCODER_3B 4

// #define N_PULSE_PER_REVOLUTION 48 // 48 counts per revolution of the motor shaft when counting both edges of both channels
#define N_PULSE_PER_REVOLUTION 24 // 24 counts per revolution of the motor shaft when counting both edges of a single channel
#define GEAR_RATIO 98.78  // 98.78:1 metal spur gearbox


// Global variables
// I2C with Motor Driver
int MotorSpeed1 = 0;  // -800 ~ +800
int DSpeed = 10;
// Encoders
int PreviousPluseCounter1 = 0;
int PluseCounter1 = 0;
int pluseCounter2 = 0;
int pluseCounter3 = 0;
// Serial
char c;
const byte NumChars = 32; // Set a read-only value of 32 bytes and assign it to NumChars
char ReceivedChars[NumChars]; // define variable of type character as an array of size NumChars
boolean NewData = false;  // variable to control interaction between my functions
char EndMarker = '\n';
const byte NewString = 32;
char Access[NewString];


void EncoderAInterrupt() {
  if (digitalRead(ENCODER_1A) == HIGH) {  // rising edge
    PluseCounter1 += (digitalRead(ENCODER_1B) == LOW) ? 1 : -1;
  }
  else {  // falling edge
    PluseCounter1 += (digitalRead(ENCODER_1B) == HIGH) ? 1 : -1;
  }
}


void setup()
{
  // serial setting
  Serial.begin(115200);
  HWSERIAL.begin(115200);

  // motor encoder
  pinMode(ENCODER_1A, INPUT);
  pinMode(ENCODER_1B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), EncoderAInterrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_1B), EncoderBInterrupt, CHANGE);

  // motor pin setting
  WIRE.begin(); // I2C with master mode
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  // Configure motor 1
  mc.setMaxAcceleration(1, 140);
  mc.setMaxDeceleration(1, 300);
  // Configure motor 2
  mc.setMaxAcceleration(2, 200);
  mc.setMaxDeceleration(2, 300);
  // Configure motor 3
  mc.setMaxAcceleration(3, 80);
  mc.setMaxDeceleration(3, 300);

  Serial.println("INFO: Teensy is ready.");
}


void loop()
{
  start_input();
  motor_ctl(); //function to control speed on key press

  // motor position
  if (PreviousPluseCounter1 != PluseCounter1) {
    HWSERIAL.printf("p%d\n", PluseCounter1);
    HWSERIAL.printf("r%f\n", PluseCounter1 / float(N_PULSE_PER_REVOLUTION) / GEAR_RATIO);
    PreviousPluseCounter1 = PluseCounter1;
  }
}

void start_input() {
  static byte ndx = 0;
  char rc;  // receiver

  while (HWSERIAL.available() > 0 && NewData == false) {
    rc = HWSERIAL.read();

    if (rc != EndMarker) {
      ReceivedChars[ndx++] = rc;
      if (ndx >= NumChars) {
        ndx = NumChars - 1;
      }
    }
    else {
      ReceivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      NewData = true;
    }
  }
}

void set_speed(int motor, int speed) {
  mc.setSpeed(motor, -speed);
}

void motor_ctl() {  //function to allow user input to control motor, display current speed, and tell user when max or minimum speed is reached
  c = ReceivedChars[0];
  if (c == '+' && NewData == true) {
    if (MotorSpeed1 <= (800 - DSpeed)) { //Do this when current motor speed is less than or equal to specified cut off value
      MotorSpeed1 += DSpeed;
      Serial.print("Motor speed is ");
      Serial.println(MotorSpeed1);
    }
    else if (((800 - DSpeed) < MotorSpeed1) && (MotorSpeed1 <= 800)) {  //Do this when motor speed is above cut off value
      MotorSpeed1 = 800;
      Serial.print("Motor speed is ");
      Serial.println(MotorSpeed1);
    }
    else {  //Do this when key is pressed while motor already at max speed
      Serial.println("Giving her all she's got");
    }
    NewData = false;
  }

  else if (c == '-' && NewData == true) { //On - key press
    if (DSpeed <= MotorSpeed1) { //Do this when speed is above cut off value
      MotorSpeed1 = MotorSpeed1 - DSpeed;
      Serial.print("Motor speed is ");
      Serial.println(MotorSpeed1);
    }
    else if ((-800 <= MotorSpeed1) && (MotorSpeed1 < DSpeed)) {  //Do this when speed is
      //below cut off value
      MotorSpeed1 = 0;
      Serial.print("Motor speed is ");
      Serial.println(MotorSpeed1);
    }
    else {  //When motor is already at lowest possible
      Serial.println("Reverse speed not possible");
    }
    NewData = false;
  }

  else if (c == 't' && NewData == true) { //On 't' input
    int newSpeed;
    char *p = ReceivedChars + 1; //start reading string at second value
    strcpy(Access, p); // copy string value from ReceivedChars to access starting at second value
    int input;
    input = atoi(Access); //integer value from string Access

    if ((-800 <= input) && (input <= 800)) {  // if integer value of array is above 0 and equal to or less than 800. or if the result of comparing string Access to the string "0" is 0/the same.
      newSpeed = input; //convert new array string to an integer value
      MotorSpeed1 = newSpeed;
      Serial.print("Motor speed is ");
      Serial.println(MotorSpeed1);
      NewData = false;
    }
    else {
      Serial.println("Invalid Input");
      NewData = false;
    }
  }

  else {
    set_speed(1, MotorSpeed1); //Commit new speeds given by
    //output of above statements
    NewData = false;
  }
}

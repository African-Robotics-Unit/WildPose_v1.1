#include <Motoron.h>

#define HWSERIAL Serial3

// Motor driver settings
MotoronI2C mc(16);  // Motoron controller
#define WIRE Wire // using I2C 0
#define N_MOTOR 3 // the number of motors
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
int MotorSpeeds[] = {0, 0, 0};  // -800 ~ +800
int DSpeed = 50;
// Encoders
int PreviousPluseCounters[] = {0, 0, 0};
int PluseCounters[] = {0, 0, 0};
// Serial
char c;
const byte NumChars = 32; // Set a read-only value of 32 bytes and assign it to NumChars
char ReceivedChars[NumChars]; // define variable of type character as an array of size NumChars
boolean NewData = false;  // variable to control interaction between my functions
char EndMarker = '\n';
const byte NewString = 32;
char Access[NewString];


void Encoder1AInterrupt() {
  if (digitalRead(ENCODER_1A) == HIGH) {  // rising edge
    PluseCounters[0] += (digitalRead(ENCODER_1B) == LOW) ? 1 : -1;
  }
  else {  // falling edge
    PluseCounters[0] += (digitalRead(ENCODER_1B) == HIGH) ? 1 : -1;
  }
}
void Encoder2AInterrupt() {
  if (digitalRead(ENCODER_2A) == HIGH) {  // rising edge
    PluseCounters[1] += (digitalRead(ENCODER_1B) == LOW) ? 1 : -1;
  }
  else {  // falling edge
    PluseCounters[1] += (digitalRead(ENCODER_1B) == HIGH) ? 1 : -1;
  }
}
void Encoder3AInterrupt() {
  if (digitalRead(ENCODER_3A) == HIGH) {  // rising edge
    PluseCounters[2] += (digitalRead(ENCODER_1B) == LOW) ? 1 : -1;
  }
  else {  // falling edge
    PluseCounters[2] += (digitalRead(ENCODER_1B) == HIGH) ? 1 : -1;
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
  attachInterrupt(digitalPinToInterrupt(ENCODER_1A), Encoder1AInterrupt, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(ENCODER_1B), EncoderBInterrupt, CHANGE);
  pinMode(ENCODER_2A, INPUT);
  pinMode(ENCODER_2B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_2A), Encoder2AInterrupt, CHANGE);
  pinMode(ENCODER_3A, INPUT);
  pinMode(ENCODER_3B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_3A), Encoder3AInterrupt, CHANGE);

  // motor pin setting
  WIRE.begin(); // I2C with master mode
  mc.reinitialize();
  mc.disableCrc();
  mc.clearResetFlag();
  // Configure motors
  for (int i = 0; i < N_MOTOR; i++) {
    mc.setMaxAcceleration(i, 140);
    mc.setMaxDeceleration(i, 300);
  }

  Serial.println("INFO: Teensy is ready.");
}


void loop()
{
  start_input();
  motor_ctl(); //function to control speed on key press

  // motor position
  for (int i = 0; i < N_MOTOR; i++) {
    if (PreviousPluseCounters[i] != PluseCounters[i]) {
      HWSERIAL.printf("p%d%d\n", i, PluseCounters[i]);
      HWSERIAL.printf("r%d%f\n", i, PluseCounters[i] / float(N_PULSE_PER_REVOLUTION) / GEAR_RATIO);
      PreviousPluseCounters[i] = PluseCounters[i];
    }
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
  
  if (c == 't' && NewData == true) { //On 't' input
    int newSpeed;
    c = ReceivedChars[1];
    char *p = ReceivedChars + 2; //start reading string at second value
    strcpy(Access, p); // copy string value from ReceivedChars to access starting at second value
    int input = atoi(Access); //integer value from string Access
    int motor_id = c - '0';

    if ((-800 <= input) && (input <= 800)) {  // if integer value of array is above 0 and equal to or less than 800. or if the result of comparing string Access to the string "0" is 0/the same.
      newSpeed = input; //convert new array string to an integer value
      MotorSpeeds[motor_id] = newSpeed;
      NewData = false;
    }
    else {
      Serial.println("Invalid Input");
      NewData = false;
    }
  }

  else if (c == 'r' && NewData == true) { // Reset
    char *p = ReceivedChars;
    strcpy(Access, p);

    if (strcmp(Access, "reset") == 0) {
      for (int i = 0; i < N_MOTOR; i++) {
        PreviousPluseCounters[i] = 0;
        PluseCounters[i] = 0;
        MotorSpeeds[i] = 0;
      }
      Serial.println("Reset.");
      NewData = false;
    }
    else {
      NewData = false;
    }
  }

  else {
    for (int i = 0; i < N_MOTOR; i++ ) {
      set_speed(i+1, MotorSpeeds[i]); //Commit new speeds given by
      Serial.printf("Motor%d speed is %d\n", i, MotorSpeeds[i]);
    }
    //output of above statements
    NewData = false;
  }
}

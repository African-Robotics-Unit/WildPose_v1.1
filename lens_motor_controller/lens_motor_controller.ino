// #define HWSERIAL Serial1

// void setup() {
//   // put your setup code here, to run once:
//   Serial.begin(115200);
//   HWSERIAL.begin(115200, SERIAL_8N1); // sets data, parity(N:none, E:even, O:odd), and stop bits
// }

// void loop() {
//   // put your main code here, to run repeatedly:
//   int incomingByte;

//   if (Serial.available() > 0) {
//     incomingByte = Serial.read();
//     Serial.print("USB received: ");
//     Serial.println(incomingByte, DEC);
//     HWSERIAL.print("USB received:");
//     HWSERIAL.println(incomingByte, DEC);
//   }
//   if (HWSERIAL.available() > 0) {
//     incomingByte = HWSERIAL.read();
//     Serial.print("UART received: ");
//     Serial.println(incomingByte, DEC);
//   }
// }



#define HWSERIAL Serial1


int motor = 3; //define motor as pin 3
int pwm_speed = 127; //initial speed of motor
int value = 10; //value to increment or decrement motor speed by on key press

char c;

const byte numChars = 32; //Set a read-only value of 32 bytes and assign it to numChars
char receivedChars[numChars]; //define variable of type character as an array of size numChars

boolean newData = false; //variable to control interaction between my functions

const byte newString = 32;
char access[newString];


void setup()
{
  // serial setting
  Serial.begin(115200);
  HWSERIAL.begin(115200);
  // motor pin setting
  pinMode(motor, OUTPUT); //initialize motor
  analogWrite(motor, pwm_speed); //start motor at set value

  Serial.println("<Arduino is ready>");
}

void loop()
{
  start_input();
  motor_ctl(); //function to control speed on key press
}

void start_input() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;  // receiver

  while (HWSERIAL.available() > 0 && newData == false) {
    rc = HWSERIAL.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void motor_ctl() //function to allow user input to control motor, display current speed,
//and tell user when max or minimum speed is reached
{
  c = receivedChars[0];
  if (c == '+' && newData == true)
  {
    if (pwm_speed <= (255 - value)) //Do this when current motor speed is less than
      //or equal to specified cut off value
    {
      pwm_speed = pwm_speed + value;
      Serial.print("Motor speed is ");
      Serial.println(pwm_speed);
    }
    else if ((pwm_speed > (255 - value)) && (pwm_speed < 255)) //Do this when motor
      //speed is above
      //cut off value
    {
      pwm_speed = 255;
      Serial.print("Motor speed is ");
      Serial.println(pwm_speed);
    }
    else //Do this when key is pressed while motor already at max speed
    {
      Serial.println("Giving her all she's got");
    }
    newData = false;
  }

  else if (c == '-' && newData == true) //On - key press
  {
    if (pwm_speed >= value) //Do this when speed is above cut off value
    {
      pwm_speed = pwm_speed - value;
      Serial.print("Motor speed is ");
      Serial.println(pwm_speed);
    }
    else if ((pwm_speed < value) && (pwm_speed > 0)) //Do this when speed is
      //below cut off value
    {
      pwm_speed = 0;
      Serial.print("Motor speed is ");
      Serial.println(pwm_speed);
    }
    else //When motor is already at lowest possible
    {
      Serial.println("Reverse speed not possible");
    }
    newData = false;
  }

  else if (c == 't' && newData == true) //On 't' input
  {
    int newSpeed;
    char *p = receivedChars + 1; //start reading string at second value
    strcpy(access, p); // copy string value from receivedChars to access starting at second value
    int input;
    input = atoi(access); //integer value from string access

    if (((input > 0) && (input <= 255)) || (strcmp(access, "0") == 0)) // if integer value of array is above 0 and equal to or less than 255.
    //or if the result of comparing string access to the string "0" is 0/the same.
    {
      newSpeed = input; //convert new array string to an integer value
      pwm_speed = newSpeed;
      Serial.print("Motor speed is ");
      Serial.println(pwm_speed);
      newData = false;
    }
    else
    {
      Serial.println("Invalid Input");
      newData = false;
    }
  }

  else
  {
    analogWrite(motor, pwm_speed); //Commit new speeds given by
    //output of above statements
    newData = false;
  }
}
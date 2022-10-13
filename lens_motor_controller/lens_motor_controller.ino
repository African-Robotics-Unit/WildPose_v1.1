#define HWSERIAL Serial1

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  HWSERIAL.begin(115200, SERIAL_8N1); // sets data, parity(N:none, E:even, O:odd), and stop bits
}

void loop() {
  // put your main code here, to run repeatedly:
  int incomingByte;

  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    Serial.print("USB received: ");
    Serial.println(incomingByte, DEC);
    HWSERIAL.print("USB received:");
    HWSERIAL.println(incomingByte, DEC);
  }
  if (HWSERIAL.available() > 0) {
    incomingByte = HWSERIAL.read();
    Serial.print("UART received: ");
    Serial.println(incomingByte, DEC);
  }
}

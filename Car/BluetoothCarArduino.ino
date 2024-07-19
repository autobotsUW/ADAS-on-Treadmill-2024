#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial bluetooth(2, 3); //RX | TX
const int motorSpeedPin = 6; // ENA for PWM speed control
Servo steeringServo;
int speed = 0;
unsigned long angle = 100;
int lastAngle = 0;
unsigned long tempsServo = 0;


void setup() {
  bluetooth.begin(9600);
  Serial.begin(9600);
  // Set the motor control pins as outputs
  pinMode(motorSpeedPin, OUTPUT);
  steeringServo.attach(11);
}


void loop() {
  //Set timer
  unsigned long previousTime, currentTime = 0;
  const unsigned long interval = 500;

  // Check if any data available to read
  if (bluetooth.available()) {
    String receivedMessage = "";
    // Read characters until ']' is found
    while (bluetooth.available()) {
      char receivedCharacter = bluetooth.read();
      if (receivedCharacter == ']') {
        break;
      }
      receivedMessage = receivedMessage + receivedCharacter  ;
    }   
    if (receivedMessage[0] == '[') {
      receivedMessage.remove(0, 1);
      int commaIndex = receivedMessage.indexOf(',');
      if (commaIndex > 0) {
        String speedString = receivedMessage.substring(0, commaIndex);
        String angleString = receivedMessage.substring(commaIndex + 1);
        int newSpeed = speedString.toInt();
        int newAngle = angleString.toInt();
   
       //Verification
        if (newSpeed >= 0 && newSpeed <= 150){
         speed = newSpeed;
        }
        if (newAngle >= 45 && newAngle <= 155){
         angle = newAngle;
        }
        previousTime = millis();
      }

    }
    if (angle != lastAngle){
      // Control the angle
      tempsServo =angle*1000/180+1000;
      steeringServo.writeMicroseconds(int(tempsServo));
      lastAngle=angle;
      delay(15);  
    }
      
  }

  // Initial value if no new values received
  currentTime = millis();
  if (currentTime - previousTime > interval){
    speed = 0;
    angle = 100;
  }
  // Control speed with PWM
  analogWrite(motorSpeedPin, speed);

}



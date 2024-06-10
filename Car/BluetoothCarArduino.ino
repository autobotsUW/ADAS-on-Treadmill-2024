#include <SoftwareSerial.h>
#include <Servo.h>

SoftwareSerial bluetooth(2, 3); //TX | RX, after we should use pin 0 and 1

// Define the motor control pins
const int motorSpeedPin = 6; // ENA for PWM speed control


Servo steeringServo;

int speed = 0;
int angle = 100;


void setup() {
  Serial.begin(9600); 
  bluetooth.begin(9600); 

  // Set the motor control pins as outputs
  pinMode(motorSpeedPin, OUTPUT);


  steeringServo.attach(10);
}


void loop() {

  unsigned long previousTime, currentTime = 0;
  const unsigned long interval = 2000;
  

  // Check if there's any data available to read
  // Serial.println("1");
  if (bluetooth.available()) {
    String receivedMessage = ""; // Initialize an empty string

    // Read characters until the closing bracket ']' is found
    while (bluetooth.available()) {
      char receivedCharacter = bluetooth.read();
      // Serial.println(receivedCharacter);
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

        // Convert the string values to integers
        int newSpeed = speedString.toInt();
        int newAngle = angleString.toInt();
      
  
    
       //Verification
      if (newSpeed >= 0 && newSpeed <= 150){
         speed = newSpeed;
     }

       if (newAngle >= 55 && newAngle <= 145){
         angle = newAngle;
      }


        Serial.print("Vitesse : ");
        Serial.println(speed);
        Serial.print("Angles : ");
        Serial.println(angle);

        previousTime = millis();

       }

      }

    // Control the angle
    steeringServo.write(angle);

      
    }

    currentTime = millis();
    if (currentTime - previousTime > interval){
      speed = 0;
      angle = 90;
    }
                // Control speed with PWM
         analogWrite(motorSpeedPin, speed);

 
    

   

}



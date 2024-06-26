#include <Servo.h>
int vibrationmotorPin=10;

void setup() {
 Serial.begin(57600); //defining a wanted baud rate ----  i dont know if its necessary here
 pinMode(vibrationmotorPin,OUTPUT); //define the vibration motor as a one who accepts an order from arduino
}

void loop() {
 if (Serial.available() >0){       
      String(command) = Serial.readStringUntil('\n'); //the communication between python and arduino works by strings
      if (command.startsWith("activate_vibrationmotor")){ 
        //Serial.println("vib_mot");
        digitalWrite(vibrationmotorPin,HIGH);
        delay(3000); // activiating the vibration motor for 3000ms, can be determined to any wanted value
        digitalWrite(vibrationmotorPin,LOW);
        Serial.println("vibration motor activated for 3 sec"); // sending the needed string to python
      } else {
        Serial.println("vibration motor wasn't activated");
      }
    }
  }    


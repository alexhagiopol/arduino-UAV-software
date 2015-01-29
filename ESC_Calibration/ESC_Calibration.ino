/*
This file is used to calibrate Electronic Speed Controllers (ESCs) by setting the 
Servo library microseconds values which correspond to minimum and maximum speeds. 
*/

#include <Servo.h>

#define MAX_SIGNAL 180
#define MIN_SIGNAL 0
#define motorPin1 6
#define motorPin2 5
#define motorPin3 4
#define motorPin4 3

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;

void setup() {
  Serial.begin(9600);
  Serial.println("Program begin...");
  Serial.println("This program will calibrate the ESC.");

  motor1.attach(motorPin1);
  motor2.attach(motorPin2);
  motor3.attach(motorPin3);
  motor4.attach(motorPin4);

  Serial.println("Now writing maximum output.");
  Serial.println("Turn on power source, then wait 2 seconds and press any key.");
  motor1.write(MAX_SIGNAL);
  motor2.write(MAX_SIGNAL);
  motor3.write(MAX_SIGNAL);
  motor4.write(MAX_SIGNAL);
  
  // Wait for input
  while (!Serial.available());
  Serial.read();
  delay(5000);
  // Send min output
  Serial.println("Sending minimum output");
  motor1.write(MIN_SIGNAL);
  motor2.write(MIN_SIGNAL);
  motor3.write(MIN_SIGNAL);
  motor4.write(MIN_SIGNAL);

}

void loop() {  

}

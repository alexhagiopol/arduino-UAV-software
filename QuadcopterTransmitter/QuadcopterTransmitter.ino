/* Quadcopter Transmitter Firmware
This program is executed by an Arduino Uno board in/on a manual controller.
It accepts input from joysticks and knobs and then sends commands to the Arduino Mega aboard
a quadcopter equipped with a DJI Naza flight controller. Many of the microseconds values you 
see are chosen based on the way that the DJI Naza responds to the servo.writeMicroseconds() command.

Last revision on 6/12/14 by Alex Hagiopol
*/

#include <SPI.h>
//Download the RF24 library from https://github.com/maniacbug/RF24 to make the following inclusions work:
#include <nRF24L01.h>   
#include <RF24.h>       
#include <RF24_config.h>
#include "NazaDecoderLib.h"

//digital pins
int buttonPin = 3;
int blueLedPin = 4;
int yellowLedPin = 5;
int redLedPin = 6;
int buzzerPin = 7;
int cePin = 9;
int csnPin = 10;
//analog pins
int leftStickX = 3;
int leftStickY = 4;
int rightPot = 5;
int rightStickX = 1;
int rightStickY = 2;
int rightStickButton = 0;
//other variables
boolean armed = false;
const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
RF24 radio(cePin, csnPin); // Create a Radio
int flightControls[4];
/*The use of flightControls changes when the state of the quadcopter changes from armed 
to disarmed as indicated by the state of boolean armed. 

When disarmed, the transmitter sends the array [0,0,0,X] where X is the setting that corresponds 
to either Naza GPS Mode, Naza Attitude Hold Mode, or Naza Manual Mode. These three modes are represented 
by signals of 1855 microseconds, 1525 microseconds, and 1185 microseconds respectively. Switching between the modes
happens when the user presses the button corresponding to buttonPin. 

When armed, the transmitter sends the array [T,A,R,E] where T is the throttle setting, A is the aileron setting,
R is the rudder setting, and E is the elevator setting. All of the settings are represented by signals between
1200 and 1800 microseconds.

The receiver is also programed to have armed and disarmed modes in which it listens for signals of the types described 
above. To switch modes from armed to disarmed, the transmitter waits for the user to hold down the right stick button. When this happens,
bollean armed becomes true, and the transmitter sends the array [1220, 1220, 1220, 1220]. The receiver interprets this array as the
signal to arm itself, begin flight, and listen for arrays of the type [T,A,R,E].
*/


void setup()  
{
  pinMode(leftStickX, INPUT);    
  pinMode(leftStickY, INPUT);
  pinMode(rightPot, INPUT);
  pinMode(rightStickX, INPUT); //Don't use rightStickY because we have no use for it yet.
  pinMode(rightStickButton, INPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(yellowLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  
  digitalWrite(blueLedPin, LOW);   //blue on = GPS Mode engaged
  digitalWrite(yellowLedPin, LOW); //yellow on = ATTI Mode engaged
  digitalWrite(redLedPin, HIGH);   //red on = Manual Mode engaged
  
  flightControls[0] = 0; 
  flightControls[1] = 0;
  flightControls[2] = 0;
  flightControls[3] = 1185;
  //1185 sets to "Manual" Mode
  //1525 sets DJI Naza to "Attitude Hold" mode
  //1855 sets to "GPS" mode

  
  Serial.begin(9600);
  delay(1);
  radio.begin();
  radio.openWritingPipe(pipe);
  
  while (armed == false)
  {
   //NON-ARMED COMMUNICATIONS
   radio.write(flightControls, sizeof(flightControls)); 
   
   //DJI NAZA MODE SELECTION
   if (digitalRead(buttonPin) == HIGH) //Crude software debouncing part 1
   {
     delay(100);
     if (digitalRead(buttonPin) == HIGH) //Crude software debouncing part 2
     {
       delay(300);
       if (flightControls[3] == 1185) //If we are in Manual mode, switch to GPS mode
       {
         flightControls[3] = 1855;
         digitalWrite(redLedPin, LOW);
         digitalWrite(blueLedPin, HIGH);
       }
       else if (flightControls[3] == 1855) //If we are in GPS mode, switch to Manual mode
       {
         flightControls[3] = 1185;
         digitalWrite(blueLedPin, LOW);
         digitalWrite(redLedPin, HIGH);
       }
     }      
   }
   
   //ARMING SEQUENCE 
    if (analogRead(rightStickButton) < 20) //User must hold and hear the first beep
    {
        tone(buzzerPin, 1000);
        delay(500);
        noTone(buzzerPin);
        delay(500);
        if (analogRead(rightStickButton) < 20) //User must hold and hear the second beep
        {
          tone(buzzerPin, 1000);
          delay(500);
          noTone(buzzerPin);
          delay(500);
          if (analogRead(rightStickButton) < 20) //User must hold and hear the third beep
          {
            tone(buzzerPin, 1000);
            delay(500);
            noTone(buzzerPin);
            delay(500);
            if (analogRead(rightStickButton) < 20)  //If user is still holding after three beeps, execute the arming sequence and send the arming signal to the receiver
            {   
              for (int i = 0; i < 50; i++)
              {    
                flightControls[0] = 1220; 
                flightControls[1] = 1220;
                flightControls[2] = 1220;
                flightControls[3] = 1220;
                radio.write(flightControls, sizeof(flightControls));
                delay(2);  
              }
              armed = true;                                        //Change boolean armed to exit this loop
              Serial.println("Armed.");
           }
          }
        }
    }
  }
}

void loop() //This executes during flight and is kept as short as possible.
{
  int throttle = analogRead(rightPot);
  int aileron = analogRead(leftStickX);
  int rudder = analogRead(rightStickX);
  int elevator = analogRead(leftStickY);
  //Serial.println(aileron);
  flightControls[0] = map(throttle, 0, 1023, 1220, 1720); 
  flightControls[1] = map(aileron,0,1023,1220,1720); 
  flightControls[2] = map(rudder,0,1023,1350,1650);
  flightControls[3] = map(elevator,0,1023,1220,1720);
  //Serial.println("throttle =" + (String) flightControls[0] + " aileron = " + (String) flightControls[1] + " rudder  = " + (String) flightControls[2] + " elevator =" + (String) flightControls[3]);
  radio.write(flightControls, sizeof(flightControls));
}



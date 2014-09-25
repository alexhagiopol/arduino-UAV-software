/*Quadcopter Receiver Firmware
This program is executed by an Arduino Mega aboard a quadcopter equipped with a DJI Naza. The program
accepts joystick and knob control signals via RF communications. It then sends these controls to the DJI Naza
which controls the quadcopter. Many of the microseconds values you see are chosen based on the way that the DJI 
Naza responds to the servo.writeMicroseconds() command. If you want to add flight controls for autonomous behavior,
you have to add them to this script.

Last revision on 8/06/14 by Alex Hagiopol
*/

#include <Servo.h>
#include <SPI.h>
//Download and import the RF24 library from https://github.com/maniacbug/RF24 to make the following inclusions work:
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>
//Download and import the naza decoder library from http://www.rcgroups.com/forums/showthread.php?t=1995704
#include <NazaDecoderLib.h>

//DJI Naza accepts input in the form of commands from the Arduino Servo Library
Servo throttle;
Servo rudder;
Servo aileron;
Servo elevator;
Servo gear;

int csnPin = 10;
int cePin = 9;
int throttlePin = 5;
int rudderPin = 6;
int elePin = 4;
int ailPin = 3;
int gearPin = 7; 
int blueLedPin = 8;
int yellowLedPin = 11;
int redLedPin = 12;
int megaLedPin = 13;
int throttleSetting = 1220; //1220 us correspnds to 0% thrust on Naza. We want to always know what the throttle setting is for safety reasons.
int gearSetting = 1855;
int lostRadioCount = 0;
uint32_t currTime, attiTime; //Left over from Naza Decoder test code. Probably useless, but keeping out of superstition
boolean armed = false;

const uint64_t pipe = 0xE8E8F0F0E1LL; // Define the transmit pipe
RF24 radio(cePin,csnPin); // Create a Radio
int flightControls[4];  // 4 element array holding Joystick readings

//********************TARGET LOCATION*******************
double targetLat = 35.0;
double targetLon = -86.0;

void setup()
{
  //Attach servos to appropriate pins
  throttle.attach(throttlePin);
  rudder.attach(rudderPin);  
  aileron.attach(ailPin);
  elevator.attach(elePin);
  gear.attach(gearPin);
  pinMode(megaLedPin, OUTPUT);
  pinMode(blueLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);
  
  //Initial throttle controls: Zero thrust, all other controls neutral.
  //Imperfections in system necessitate using 1470 as opposed to 1500 in some cases
  flightControls[0] = 1220;
  flightControls[1] = 1470;
  flightControls[2] = 1500;
  flightControls[3] = 1470;
  throttleSetting = flightControls[0]; //Redundant, but done mostly for code clarity.
  throttle.writeMicroseconds(throttleSetting);
  aileron.writeMicroseconds(flightControls[1]);
  rudder.writeMicroseconds(flightControls[2]);
  elevator.writeMicroseconds(flightControls[3]);
  gear.writeMicroseconds(gearSetting); //1525 sets dji naza to "Attitude Hold" mode
                                //1185 sets to "Manual" Mode
                                //1855 sets to "GPS" mode
  //RADIO INITIALIZATION
  Serial.begin(9600); //???? Does this need to be 115200? Does the RF Transmitter library depend on Serial? (Guess no)
  Serial3.begin(115200);
  delay(100);
  radio.begin();
  radio.openReadingPipe(1,pipe);
  radio.startListening();;  
  
  //Serial.println("Setup starting. Waiting for gear command.");
  //GROUND SETUP SEQUENCE
  while (armed == false)
  {
    Serial.println("Disarmed." + (String) flightControls[0] + " " + (String) flightControls[1] + " " + (String) flightControls[2] + " " + (String) flightControls[3]);
    //FLASHING LED TO SHOW DISARMED STATE
    delay(50);
    digitalWrite(megaLedPin, HIGH);
    delay(50);
    digitalWrite(megaLedPin, LOW);
    
    //LISTENING FOR ARMING SIGNAL AND NAZA MODE SELECTION
    if (radio.available())
    {
      bool done = false;
      while (!done)
      {
        // Fetch the data payload
        done = radio.read(flightControls, sizeof(flightControls));
      }
    }
    
    Serial.println("Disarmed");
    
    //****************************ACQUIRE & REPORT GPS DATA***********************************
    if(Serial3.available())
    {
      uint8_t decodedMessage = NazaDecoder.decode(Serial3.read()); 
      switch (decodedMessage)
      {
        case NAZA_MESSAGE_GPS:
          Serial.print("Lat: "); Serial.print(NazaDecoder.getLat(), 7);
          Serial.print(", Lon: "); Serial.print(NazaDecoder.getLon(), 7);
          Serial.print(", Alt: "); Serial.print(NazaDecoder.getGpsAlt(), 7);
          Serial.print(", Fix: "); Serial.print(NazaDecoder.getFixType());
          Serial.print(", Sat: "); Serial.println(NazaDecoder.getNumSat());
          break;
        case NAZA_MESSAGE_COMPASS:
          Serial.print("Heading: "); Serial.println(NazaDecoder.getHeadingNc(), 2);
          break;
      }
    }
    //*******************************************************************************
    
        
    if (flightControls[3] == 1185) //If we are in Manual mode, switch lights to GPS mode
       {
         gearSetting  = 1185;
         digitalWrite(redLedPin, LOW);
         digitalWrite(blueLedPin, HIGH);
         //Serial.println("Gear set to 1185.");
       }
    else if (flightControls[3] == 1855) //If we are in GPS mode, switch lights to Manual mode
       {
         gearSetting = 1855;
         digitalWrite(blueLedPin, LOW);
         digitalWrite(redLedPin, HIGH);
         //Serial.println("Gear set to 1855.");
       }
        
    //ARMING SIGNAL RECEIVED
    if ((flightControls[0] == 1220) && (flightControls[1] == 1220) && (flightControls[2] == 1220) && (flightControls[3] == 1220))
    {      
      for (int i = 0; i < 50; i++)
      {
        throttle.writeMicroseconds(flightControls[0]);
        aileron.writeMicroseconds(1850);
        rudder.writeMicroseconds(flightControls[2]); 
        elevator.writeMicroseconds(flightControls[3]);
        delay(20);
      }
      digitalWrite(megaLedPin, LOW);
      armed = true;
      Serial.println("Armed.");
    }
    
    //Keep Naza thinking everything is OK, but do not send actual controller values.
    throttle.writeMicroseconds(1220);
    aileron.writeMicroseconds(1470);
    rudder.writeMicroseconds(1500);
    elevator.writeMicroseconds(1470);
    gear.writeMicroseconds(gearSetting);
    
  }
}

void loop()
{
  //Serial.println("Armed." + (String) flightControls[0] + " " + (String) flightControls[1] + " " + (String) flightControls[2] + " " + (String) flightControls[3]);
  if (radio.available())
  {
    lostRadioCount = 0;
    bool done = false;
    while (!done)
    {
      // Fetch the data payload
      done = radio.read(flightControls, sizeof(flightControls));
     }
  }
  else
  {
    lostRadioCount = lostRadioCount + 1; 
    if (lostRadioCount > 30000)
    {
      //Serial.println("EMERGENCY");
      emergencyLanding();
    }
  }
  throttleSetting = flightControls[0]; //We store the value of the throttle for safety during emergencyLanding()
  //Serial.println("Armed. t =" + (String) throttleSetting + " a = " + (String) flightControls[1] + " r = " + (String) flightControls[2] + " e = " + (String) flightControls[3]);
  throttle.writeMicroseconds(flightControls[0]);
  aileron.writeMicroseconds(flightControls[1]);
  rudder.writeMicroseconds(flightControls[2]); 
  elevator.writeMicroseconds(flightControls[3]);
}

void emergencyLanding()
{
  //Serial.println("in function");
  armed = false;
  aileron.writeMicroseconds(1470);
  rudder.writeMicroseconds(1500);
  elevator.writeMicroseconds(1470);
  //Serial.println("neutralized flightControls");
  
  int throttleDiff = throttleSetting - 1200;
  //Serial.println("throttleDiff calculated = " + (String) throttleDiff);

  for (throttleDiff; throttleDiff > 0; throttleDiff--)
  {
    throttleSetting = 1200 + throttleDiff;
    throttle.writeMicroseconds(throttleSetting);
    //Serial.println("Landing. t = " + (String) throttleSetting);
    delay(10);
  }
  
  //Serial.println("throttle off");
    
  while (armed == false)
  {
    delay(250);
    digitalWrite(megaLedPin, HIGH);
    delay(250);
    digitalWrite(megaLedPin, LOW);
    if (radio.available())
    {
      bool done = false;
      while (!done)
      {
        // Fetch the data payload
        done = radio.read(flightControls, sizeof(flightControls));
      }
    }
    throttleSetting = flightControls[0];
    Serial.println("Crash landed. Waiting for arming sequence. t = " + (String) throttleSetting);
    if ((throttleSetting == 1220) && (flightControls[1] == 1220) && (flightControls[2] == 1220) && (flightControls[3] == 1220))
    {      
      throttle.writeMicroseconds(throttleSetting);
      aileron.writeMicroseconds(flightControls[1]);
      rudder.writeMicroseconds(flightControls[2]); 
      elevator.writeMicroseconds(flightControls[3]);
      lostRadioCount = 0;
      digitalWrite(megaLedPin, LOW);
      armed = true;
    }
  }

}

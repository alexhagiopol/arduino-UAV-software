/*********************************************************************************************************************************************************
NavigateObstacles v0.5 by Alex Hagiopol
Last updated 2/15/14

This program is used by an Arduino Uno R3 robot to navigate obstacles. This program assumes a smooth, flat environment with vertical obstacles. Example 
real-world environments in which this assumption holds include offices. 
*********************************************************************************************************************************************************/

/*********************************************************************************************************************************************************
INITIALIZATION OPERATIONS ARE HERE  
*********************************************************************************************************************************************************/

#include <Servo.h>  //Import relevant libraries.
#include <EEPROM.h>
#include <math.h>
#include <SoftwareSerial.h>

Servo leftServo;
Servo rightServo;
int button1Presses = 0;                  //Variable used to keep track of user button presses. Useful for ON/OFF effect.
int myDirection = 0;                     //Robot's turning direction. 0 = Clockwise. 1 = Counterclockwise.
float currentSpeed = 0.0;                //Normalized current servo speed on a 0 to 1 scale. 
unsigned long travelBeginTime = 0UL;     //Variables used to determine travel times.
unsigned long travelEndTime = 0UL;
int dataAddress = 0;                     //Default data location in EEPROM. EEPROM has locations numbered 0 to 1023
float frontDist = 100.0;                 //Initiallly assumed distance from ultrasonic sensor to closest obstacle. 
boolean rightObstacle = false;           //Right obstacle detected by IR sensor?
boolean leftObstacle = false;            //Left obstacle detected by IR sensor?
double lastTheta = 90.0;                 //Robot's last direction of travel measured in degrees. Assumes that the robot's first heading will be "North"
int lastDist = 0;                        //Robot's last distance traveled measured in cm.

int button2Pin = 2;                      //Pin number initializations.
int rightIRin = 3;
int rightLedPin = 4;
int screenPin = 5;
int trigPin = 6;
int echoPin = 7;
int button1Pin = 8;
int leftLedPin = 9;
int leftIRin = 10;
int IRout = 11;
int leftServoPin = 12;
int rightServoPin = 13;
SoftwareSerial screenSerial = SoftwareSerial(255, screenPin); //Use screenPin to send information to LCD. Use non-existent pin 255 to receive information.

/*********************************************************************************************************************************************************
SETUP OPERATIONS ARE HERE  
*********************************************************************************************************************************************************/

void setup()
{
pinMode(IRout, OUTPUT);
pinMode(leftLedPin, OUTPUT);
pinMode(rightLedPin, OUTPUT);
pinMode(trigPin, OUTPUT);               //Trigger pin for ultrasonic sensor.
pinMode(screenPin, OUTPUT);             //Pin for serial communications to LCD.

pinMode(leftIRin, INPUT);
pinMode(rightIRin, INPUT);
pinMode(button1Pin,INPUT);  
pinMode(echoPin, INPUT);
pinMode(button2Pin, INPUT);

digitalWrite(screenPin, HIGH);          // Initialization procedure for Parallax LCD
screenSerial.begin(9600);               // Start software serial to communicate with LCD  
screenSerial.write(211);                // Set 1/4 note
screenSerial.write(216);                // Set 440 Hz scale
screenSerial.write(220);                // Play F note to indicate initialization is complete.
displayData("Hello.","");               
//deleteData();                         //Uncomment deleteData() when you wish to clear the EEPROM upon reset.
}

/*********************************************************************************************************************************************************
MAIN LOOP OPERATIONS ARE HERE  
*********************************************************************************************************************************************************/

void loop()
{
checkSurroundings();  
checkButton();
updateDistance();
if (((frontDist < 15.0) || (leftObstacle) || (rightObstacle)) && (button1Presses == 1))   //If obstacles are nearby AND the user has previously pressed the GO button...
{
  setServoSpeed(0.0);         //Decelerate to stop.
  saveData(lastDist);
  saveData((int) lastTheta); 
  displayData("Dist = " + (String) lastDist + "cm", "Dir = " + (String) (int) lastTheta + "deg"); //Update screen to reflect last maneuver
  Serial.begin(9600);
  Serial.println(lastDist);
  Serial.println(lastTheta);
  Serial.end();
  delay(100);             //Delay to make sure all of the data has been saved and displayed; LCD takes forever sometimes.
  int angleAdjustments = 0;   //Start counting the number of times the robot's orientation has been adjusted.
  if (rightObstacle) 
  {
   multiBeep(1,1);            //Beep to signal obstacles in the way.
   myDirection = 1;           //Default direction is CCW.
  }
  else if (leftObstacle)
  {
    multiBeep(1,0);
    myDirection = -1;
  }
  else if (leftObstacle && rightObstacle)
  {
    multiBeep(1,2);                      //Don't change direction when two obstacles detected. Assume that previously selected direction is as good as any other.
  }
  else
  {
   multiBeep(1,2);                       //Don't change direction when no side obstacles detected. Assume that previously selected direction is as good as any other.
  }
  while (((frontDist < 15.0) || (leftObstacle) || (rightObstacle)) && (button1Presses == 1)) //While there are obstalces nearby AND the user has pressed the GO button...
  {
    turnInPlace(myDirection, 500);       //Turn by a set amount. Knowing the set amount and the number of turns helps us calculate the angle of the resultant turn. 
    angleAdjustments = angleAdjustments + 1;
    if (myDirection < 0) //(CLOCKWISE) Update robot direction based on previous angle adjustment.
    {
      lastTheta = lastTheta - (double) angleAdjustments*(0.5/11.6)*360; //Motor speeds are different , so angle calculations are different.
    }
    else // (COUNTERCLOCKWISE)
    {
      lastTheta = lastTheta + (double) angleAdjustments*(0.5/12.0)*360;
    }
    checkSurroundings();                //Check for obstacles again after making angle adjustments.
    if (rightObstacle && !leftObstacle) //Even after turning in place, the robot may still have obstacles in front of it.
    {
      myDirection = 1;      //Alter direction without making sound
    }
    else if (leftObstacle && !rightObstacle)
    {
      myDirection = -1;     //Alter direction without making sound
    }
    //Don't change direction when two obstacles detected. Assume that previously selected direction is as good as any other.
    //Don't change direction when no side obstacles detected. Assume that previously selected direction is as good as any other.
  }  
  setServoSpeed(0.2); //resume travel
}

}

/*********************************************************************************************************************************************************
SENSOR OPERATIONS ARE HERE (checkButton, checkSurroundings)  
*********************************************************************************************************************************************************/

void checkButton()  //Checks if either button has been pressed by the user.
{
if (digitalRead(button1Pin) == HIGH)  //If Button 1 has been pressed. Button 1 is the black button used as an ON/OFF switch.
{
  delay(100);                             //allow user time to release button before executing next commands; a bad form of debouncing
  screenSerial.write(211);                // Set 1/4 note
  screenSerial.write(216);                // Set 440 Hz scale
  screenSerial.write(220);                // Play F note
  if (button1Presses == 0)                //Checks if this is an odd numbered button press.
    {
     setServoSpeed(0.2); //GO
     button1Presses = 1;           
    }
  else
    {
     setServoSpeed(0.0); //STOP
     button1Presses = 0;
     }
}
if (digitalRead(button2Pin) == HIGH) //If button 2 is pressed. Button 2 is the red button used to transmit data to the connected computer.
{
  dumpData();
  multiBeep(5,2);
}
}

void checkSurroundings()            //Check all sensors for obstacle detection.
{
  //FRONT ULTRASONIC SENSOR
  float duration;                   //variable stores the amount of time required for the sound to return to the sensor.
  for(int i = 0; i < 3; i++)        //Make 3 duration meadurements in quick succession.
  {
    digitalWrite(trigPin, LOW);     //Standard operting sequence for the HC-SR04
    delayMicroseconds(2); 
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10); 
    digitalWrite(trigPin, LOW);
    duration = duration + pulseIn(echoPin, HIGH);
    delay(5);
  }
  duration = duration / 3;           //Calculate average of three distance measurements.
  frontDist = (duration/2) / 29.1; 
  delay(5);
  
 //LEFT INFRARED SENSOR
 tone(IRout, 38000, 8);          //38000 Hz tone recommended by Parallax instruction guide. Used to send out a pulse of IR light.
 delay(1); 
 if(digitalRead(leftIRin) == 0)  //If IR light pulse detected...
 {
   leftObstacle = true;
 }
 else
 {
   leftObstacle = false;
 }
 delay(10);
 
 //RIGHT INFRARED SENSOR. Same pin used for both left and right IR LEDs. Did this to save pins on the Arduino board. 
 tone(IRout, 38000, 8);    
 delay(1);
 if(digitalRead(rightIRin) == 0)
 {
   rightObstacle = true;
 }
 else
 {
   rightObstacle = false;
 }
 delay(10);
}

/*********************************************************************************************************************************************************
ROBOT MOTION OPERATIONS ARE HERE (setServoSpeed, turnInPlace)
*********************************************************************************************************************************************************/

void setServoSpeed(float targetSpeed) //Target speed is a normalized speed value from 0 to 1. This value is then multiplied by 200 microseconds and added or subtrated 
{                                     //to or from 1500 microseconds. 1700 microseconds = max speed forward, and 1300 microseconds = max speed backward according to Parallax documentation.
  if (targetSpeed > 0)
  {
   leftServo.attach(leftServoPin);      //attach servos when desired speed is greater than 0
   rightServo.attach(rightServoPin);
   currentSpeed = targetSpeed;
   leftServo.writeMicroseconds((int)round((currentSpeed-0.02*currentSpeed)*200.0+1500.0)); //Corrects for differences between motors. Left is faster than right.
   rightServo.writeMicroseconds((int)round(1500.0-currentSpeed*200.0));
   travelBeginTime  = millis(); 
  }
  else
  {
   leftServo.detach();      
   rightServo.detach();
   currentSpeed = targetSpeed;
   leftServo.writeMicroseconds((int)round((currentSpeed-0.02*currentSpeed)*200.0+1500.0)); //Corrects for differences between motors. Left is faster than right
   rightServo.writeMicroseconds((int)round(1500.0-currentSpeed*200.0));
  }  
   
}

void turnInPlace(int direc, int time) //Time is in milliseconds. If direc is positive, go ccw. If direc is negative, go cw. Zero is considered positive.
{
  //6000 ms for a CCW 180 turn.
  //5800 ms for a CW 180 turn because of servo speed differences. This value was calculated after trial and error turning tests.
  leftServo.attach(leftServoPin);      
  rightServo.attach(rightServoPin);
  if (direc < 0)
  {
    leftServo.writeMicroseconds(1522.5); //Again, the values passed to each servo are different because the servos run at different speeds. Trial and error used to determine.
    rightServo.writeMicroseconds(1520);
    delay(time);
    leftServo.writeMicroseconds(1500);
    rightServo.writeMicroseconds(1500);
  }
  else
  {    
    leftServo.writeMicroseconds(1482);
    rightServo.writeMicroseconds(1479);
    delay(time);
    leftServo.writeMicroseconds(1500);
    rightServo.writeMicroseconds(1500);
  }
  leftServo.detach();
  leftServo.detach();
}

/*********************************************************************************************************************************************************
DATA MANIPULATION OPERATIONS ARE HERE (saveData, dumpData, deleteData, displayData, updateDistance)
*********************************************************************************************************************************************************/

void deleteData() //Clears EEPROM.
{
  for (int i = 0; i <= 1023; i++)
  {
  EEPROM.write(i,0); //Sets each value in EEPROM to zero. Each EEPROM address can hold one byte which can have a value of 0 to 255.
  }
  dataAddress = 0;   //Reset current data address to first address in EEPROM.
}

void displayData(String line1, String line2) //Shows stuff on LCD. Be careful with this. Slow serial comm makes motors stutter and misbehave. Use only when stopped?
{
  screenSerial.write(12);                 // Clear screen and begin writing to first line.        
  delay(5);                               //Delay suggested by Parallax documentation.
  screenSerial.print(line1);
  screenSerial.write(13);                 //Switch over to second line.
  screenSerial.print(line2);
}

void dumpData()                           //Transmits data from Arduino to connected Mac or PC.
{
  displayData("Transmitting..."," ");
  Serial.begin(9600);                     //Begin serial communications.
  delay(10);
  for (int i = 0; i <= 1022; i = i + 4)
  {
    int high = EEPROM.read(i);            //Reads distance traveled. Data are written to EEPROM using highbyte-lowbyte technique (see below). 
    int low = EEPROM.read(i + 1);
    int output = (high << 8) + low; //Uses inverse highbyte-lowbyte technique to interpret data from EEPROM
    Serial.println(output); //This sends the distance traveled to the Mac or PC.
    delay(5);
    high = EEPROM.read(i + 2);           //Reads the angle traveled.
    low = EEPROM.read(i + 3);
    output = (high << 8) + low;
    Serial.println(output); //This sends the angle traveled to the Mac or PC.
    delay(5);
  }
 Serial.end();
 displayData("Transmission","complete.");
}

void saveData(int data) //stores data to EEPROM. Data must be an integer.
{
  if (dataAddress <= 1023) //Highest EEPROM address on Arduino Uno is 1023 corresponding to 1024 bytes EEPROM size..
  {
  EEPROM.write(dataAddress, highByte(data));
  EEPROM.write(dataAddress + 1, lowByte(data)); //Use highbyte & lowbyte technique to split an integer into two bytes. Integers are 16 bit (two byte) pieces of data. EEPROM can only store one byte at a time.
  dataAddress = dataAddress + 2;
  }
}

void updateDistance() //Calculates the last distance traveled.
{
  if (currentSpeed > 0)
  {
  unsigned long timeDiff = (millis() - travelBeginTime); //Time traveled equals current clock time minus starting clock time.
  lastDist = (int) timeDiff / 1000.0 * 7.4; //Assume 7.4 cm/s travel speed. This travel speed was measured on a hard surface using a ruler for a currentSpeed of 0.2.
  }
}

/*********************************************************************************************************************************************************
MISC OPERATIONS ARE HERE (multiBeep)
*********************************************************************************************************************************************************/

void multiBeep(int num, int select) //Num = number of beeps. Delays your code by 200ms * num. Select can be 0,1, or 2: Left, right, or both.
{
  screenSerial.write(218);
  if (select == 2)
  {
    while (num > 0)
    {
      digitalWrite(rightLedPin, HIGH);
      digitalWrite(leftLedPin, HIGH);
      screenSerial.write(210);                // 1/16 note
      screenSerial.write(231);                // G#
      delay(100);
      digitalWrite(rightLedPin, LOW);
      digitalWrite(leftLedPin, LOW);
      num --;
    }
  }
  else if (select == 1)
  {
    while (num > 0)
    {
      digitalWrite(rightLedPin, HIGH);
      screenSerial.write(210);                // 1/16 note
      screenSerial.write(231);                // G#
      delay(100);
      digitalWrite(rightLedPin, LOW);
      num --;
    }
  }
  else
  {
   while (num > 0)
    {
      digitalWrite(leftLedPin, HIGH);
      screenSerial.write(210);                // 1/16 note
      screenSerial.write(231);                // G#
      delay(100);
      digitalWrite(leftLedPin, LOW);
      num --;
    } 
  }
}





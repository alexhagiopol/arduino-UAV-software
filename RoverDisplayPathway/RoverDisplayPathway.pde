
/*********************************************************************************************************************************************************
DisplayPathway v0.4 by Alex Hagiopol
Last updated 1/4/14

This program is meant to be used in conjunction with NavigateObstacles for the Arduino Uno R3. It accepts robot pathway data in the form of serial input. 
The program then plots the robot's pathway. One difficulty with this program is the issue of locating the correct serial port. A list of serial ports will
be printed, and the user will have to read the list and determine his/her correct serial port using trial and error. The index of the desired serial port 
will have to be entered in the line 
*********************************************************************************************************************************************************/
import processing.serial.*;
 
Serial myPort;        
int xPos = 1;         // horizontal position of the graph
int currentDataType = 0; // 0 represents a distance. 1 represents an angle. Distance is always the first type passed.
float distance = 0.0;
float angle = 0.0;
float currentX = 500;
float currentY = 500;
float finalX = 0;
float finalY = 0;
float scaleFactor = 5;

void setup ()
{

size(1000, 1000); 
println(Serial.list());
myPort = new Serial(this, Serial.list()[2], 9600);
myPort.bufferUntil('\n');
background(255);
}
 
void draw()
{  
    
}
 
void serialEvent (Serial myPort) 
{
  String inString = new String(myPort.readBytesUntil('\n'));
  println(inString);
  
  if ((inString != null) && (inString != "0"))
  { 
    inString = trim(inString);
    if (currentDataType == 0)
    {
       distance = float(inString); 
       currentDataType = 1;
    }
    else
    {
       angle = float(inString);
       finalX = currentX + cos(radians(angle)) * scaleFactor*distance;
       finalY = currentY - sin(radians(angle)) * scaleFactor*distance;
       line(currentX, currentY, finalX, finalY);
       if (distance > 5.0)
       {
       fill(0,0,0); //set text color to black
       textSize(10); //set text size to 14
       text(distance + " (cm) @ " + angle + " (deg)",currentX,currentY); //text, position
       }
       currentX = finalX;
       currentY = finalY;
       currentDataType = 0;  
    }    
  }
}


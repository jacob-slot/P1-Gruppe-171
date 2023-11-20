#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4OLED display;

#define NUM_SENSORS 5
uint16_t lineSensorValues[NUM_SENSORS];
int s1, s2, s3;

int speed = 300; // speed of turn of robot
int speedf = 350;  // speed of robot

// make difrendt thresholds for evry sensor
int threshold0;
int threshold1;
int threshold2;
int threshold3;
int threshold4;

void setup() {
  lineSensors.initFiveSensors();
  Serial.begin(9600);
  // calibrat thresholds to the flor
  readLineSensors();
 threshold0 = lineSensorValues[0]+100;
 threshold1 = lineSensorValues[1]+50;
 threshold2 = lineSensorValues[2]+50;
 threshold3 = lineSensorValues[3]+50;
 threshold4 = lineSensorValues[4]+100;
  delay(1000);
  display.clear();
  display.print(F("Press A"));
  buttonA.waitForButton();
  display.clear();
  delay(300);
  display.print(F("Go!"));
  forward(); // drive up to line after calibrate
  delay(100);
}

// Prints a line with all the sensor readings to the serial
/*void printReadingsToSerial()
{
  char buffer[80];
  sprintf(buffer, "%4d %4d %4d %4d %4d\n",
    lineSensorValues[0],
    lineSensorValues[1],
    lineSensorValues[2],
    lineSensorValues[3],
    lineSensorValues[4]
  );
  Serial.print(buffer);
}*/

// the uncalibrated line sensor reading are between 0 (very bright) and 2000 (very dark)
void readLineSensors(){
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
 // printReadingsToSerial();
}

void stop(){ // make a funtion to stop the robot
  motors.setSpeeds(0,0);
}

void forward(){ // make a funtion to drive forward the robot
  motors.setSpeeds(speedf, speedf);
}


void loop() {
readLineSensors();

if (lineSensorValues[0] > threshold0){
motors.setSpeeds(-speedf, speedf);
}
else if (lineSensorValues[4] > threshold4){
motors.setSpeeds(speedf, -speedf);
}
else if (lineSensorValues[1] > threshold1 || lineSensorValues[2] > threshold2 || lineSensorValues[3] > threshold3 ) {
forward();
}
}

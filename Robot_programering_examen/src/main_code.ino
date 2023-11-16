#include <Wire.h>
#include <Zumo32U4.h>

//LIBRARIES
Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonA buttonB;
Zumo32U4Buzzer buzzer;

//Variables for LINE SENSOR
uint16_t lineSensorValues[3];
bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

//--------------------------CHALLENGE 0--------------------------//
uint16_t listLength = 8;
char *programs[9] = 
{ 
  "         ",
  " WallStop",
  "LineTrack",
  " WallTurn",
  "SetLength",
  "   Slalom",
  "Alignment",
  " Straight"
};
//--------------------------CHALLENGE 0--------------------------//



//--------------------------CHALLENGE 1--------------------------//
uint16_t brightnessLevels[120];
float CPR = 910; 
float TC = 11.466;
int16_t scrollValue = 0;
float countsToWall = 3360; //3359 or 3374
char text[9];
//--------------------------CHALLENGE 1--------------------------//


//--------------------------CHALLENGE 5--------------------------//
//---Variables for Tape counter---///
bool tapeChange = false;            //Resets encoders when a piece of tape has been passed.
int countTape = -1;                 //Number of half laps that has been made
unsigned long currentTimeTape = 0;  //Starts counter when passing tape
//---Virables for speed---///
const int speedChallenge5 = 300;  //300 works
const int turnSpeedFastMax = speedChallenge5;
const int turnSpeedFastMin = speedChallenge5 / 4;
const float turnSpeedSlowFast = 13 / 20.0 * speedChallenge5;
const float turnSpeedSlowSlow = 8 / 20.0 * speedChallenge5;
//---Variables for sideswitch---///
bool changeState = false;  //Keeps track on when to go straight and when to turn clockwise/counterclockwise
int distanceToStraight;
int straighDistance;  //3400 standard
//---Variables for PROXY SENSORS---///
const int levelCount = 120; 
uint16_t brightnessLevel[levelCount];
//--------------------------CHALLENGE 5--------------------------//






void setup() 
{
  Wire.begin();
  Serial.begin(9600);


  //SETUP for all
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  encoders.init();
  display.init();

  //SETUP FOR challenge 0
  display.setLayout11x4();
  displayChange("PickProgram", programs[scrollValue], programs[scrollValue+1], programs[scrollValue+2]);
}



void loop() 
{
  Challenge0();
}

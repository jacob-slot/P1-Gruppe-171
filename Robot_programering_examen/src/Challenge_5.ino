#include <Wire.h>
#include <Zumo32U4.h>

//LIBRARIES
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;

//Variables for LINE SENSOR
uint16_t lineSensorValues[3];

//--------------------------CHALLENGE 5--------------------------//
//---Variables for Tape counter---///
bool tapeChange = false;            //Resets encoders when a piece of tape has been passed.
int countTape = -1;                 //Number of half laps that has been made
unsigned long currentTimeTape = 0;  //Starts counter when passing tape
//---Variables for speed---///
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
//---------------------------------------------------------------//

void setup() {
  Serial.begin(9600);

  display.clear();
  display.setLayout11x4();

  //Initialise sensors
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  proxSensors.setPulseOffTimeUs(0.5);  //Proximity sensors pulse every .5 ms. Reduces the delay in the code
  proxSensors.setPulseOnTimeUs(0.5);   // -----||------

  for (int i = 0; i < 120; i++) {
    //Makes array with values 0->119
    brightnessLevel[i] = i;
  }
  proxSensors.setBrightnessLevels(brightnessLevel, levelCount);  //Prox sensors gives back 120 levels
  display.gotoXY(1, 1);
  display.print(F("Press  A"));
  buttonA.waitForButton();
  display.clear();
  resetEncoders();
  display.gotoXY(3, 1);
  display.print("Step");
  display.gotoXY(3, 2);
  display.print("Away");

  delay(2000);
  display.clear();
}

void loop() {
  challengeFive();
}


void turnRight() {
  //Controls the motors when going clockwise around pilar
  const int distanceToStraightRight = 2200;  //Encoder-length driven before going straight
  const int straighDistanceRight = 3400;     //Encoder-length going straight

  if (proxSensors.countsRightWithRightLeds() > 65) {
    motors.setSpeeds(turnSpeedFastMax, turnSpeedFastMin);
  } else {
    motors.setSpeeds(turnSpeedSlowFast, turnSpeedSlowSlow);
  }
  distanceToStraight = distanceToStraightRight;  //Updates variables in "Void ChallengeFive"
  straighDistance = straighDistanceRight;
}

void turnLeft() {
  //Controls the motors when going counter-clockwise around pilar
  int distanceToStraightLeft = 2270;  //Encoder-length driven before going straight
  int straighDistanceLeft = 3450;     //Encoder-length going straight

  if (proxSensors.countsLeftWithLeftLeds() > 65) {
    motors.setSpeeds(turnSpeedFastMin, turnSpeedFastMax);
  } else {
    motors.setSpeeds(turnSpeedSlowSlow, turnSpeedSlowFast);
  }
  distanceToStraight = distanceToStraightLeft;  //Updates variables in "Void ChallengeFive"
  straighDistance = straighDistanceLeft;
}

void challengeFive() {
  //Function that loops
  proxSensors.read();
  tapeCount();

  if (getDistance() > distanceToStraight && tapeChange == false) {
    //When driven "distanceToStraight" it changes state
    changeState = !changeState;
    tapeChange = true;
  }

  if (tapeChange == true && getDistance() < straighDistance) {
    //When state has been changed go straight until "straighDistance"
    motors.setSpeeds(speedChallenge5, speedChallenge5);
  } else {
    if (changeState == false) {
      turnLeft();
    } else {
      turnRight();
    }
  }
}

void tapeCount() {
  lineSensors.read(lineSensorValues);

  if (currentTimeTape + 1000 < millis() && lineSensorValues[1] > 700) {
    //CHECKS MIDDLE LINE SENSOR FOR TAPE after 1 second
    currentTimeTape = millis();
    countTape++;
    //Resets encoders when hitting tape
    tapeChange = false; 
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    bip();

    display.clear();
    display.gotoXY(0, 0);
    display.print("Tape pass:");
    display.gotoXY(4, 2);
    display.print(countTape);
  }
}

float getDistance() {
  //Average encoder distance driven
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  return (countsL + countsR) / 2;
}

void bip() {
  buzzer.playNote(NOTE_A(4), 20, 15);
  delay(30);
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

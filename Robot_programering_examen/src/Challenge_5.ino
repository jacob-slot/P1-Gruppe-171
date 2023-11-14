#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;
Zumo32U4Motors motors;
Zumo32U4OLED display;
Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;

//Variables for LINE SENSOR
uint16_t lineSensorValues[3];
bool proxLeftActive;
bool proxFrontActive;
bool proxRightActive;

//-------------------VARIABLES FOR CHALLENGE FIVE (DRIVING 8's) ----------------------//
//---Variables for Tape counter---///
bool tapeChange = false;            //Resets encoders when a piece of tape has been passed.
int countTape = -1;                 //Number of half laps that has been made
unsigned long currentTimeTape = 0;  //Starts counter when passing tape
//---Virables for speed---///
int speed = 300;  //300 works
int turnSpeedFastMax = speed;
int turnSpeedFastMin = speed / 4;
float turnSpeedSlowFast = 13 / 20.0 * speed;
float turnSpeedSlowSlow = 8 / 20.0 * speed;
//---Variables for sideswitch---///
bool changeState = false;  //Keeps track on when to go straight and when to turn clockwise/counterclockwise
int distanceToStraight;
int straighDistance;  //3400 standard
//---Variables for PROXY SENSORS---///
uint8_t levelCount = 120;  //SLET
uint16_t brightnessLevel[120];
//-----------------------------------------------------------------------------------//

void setup() {
  Serial.begin(9600);

  //Initialise sensors
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  //CODE FOR CHALLENGE 5
  proxSensors.setPulseOffTimeUs(0.5);  //Proximity sensors pulse every 1ms. Reduces the delay in the code
  proxSensors.setPulseOnTimeUs(0.5);   // -----||------
  for (int i = 0; i <= 120; i++) {
    //Makes array with values 0->120
    brightnessLevel[i] = i;
  }

  proxSensors.setBrightnessLevels(brightnessLevel, levelCount);  //Prox sensors gives back 120 levels

  display.gotoXY(2, 0);
  display.print("Step");
  display.gotoXY(2, 1);
  display.print("Away");

  delay(2000);
  display.clear();
}

void loop() {
  challengeFive();

  display.gotoXY(0, 0);
  display.print(getDistance());
}

void turnRight() {
  //Controls the motors when going clockwise around pilar

  int distanceToStraightRight = 2360;  //2300
  int straighDistanceRight = 3400;
  if (proxSensors.countsRightWithRightLeds() > 65) {
    motors.setSpeeds(turnSpeedFastMax, turnSpeedFastMin);
  } else {
    motors.setSpeeds(turnSpeedSlowFast, turnSpeedSlowSlow);
  }
  distanceToStraight = distanceToStraightRight;
  straighDistance = straighDistanceRight;
}

void turnLeft() {
  //Controls the motors when going counter-clockwise around pilar

  int distanceToStraightLeft = 2420;  //2360
  int straighDistanceLeft = 3450;

  if (proxSensors.countsLeftWithLeftLeds() > 65) {
    motors.setSpeeds(turnSpeedFastMin, turnSpeedFastMax);
  } else {
    motors.setSpeeds(turnSpeedSlowSlow, turnSpeedSlowFast);
  }
  distanceToStraight = distanceToStraightLeft;
  straighDistance = straighDistanceLeft;
}

void challengeFive() {

  proxSensors.read();
  tapeCount();

  if (getDistance() > distanceToStraight && tapeChange == false) {
    //When driven "distanceToStraight" it changes state
    changeState = !changeState;
    tapeChange = true;
  }

  if (tapeChange == true && getDistance() < straighDistance) {
    //When state has been changed go straight until "straighDistance"
    motors.setSpeeds(speed, speed);
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

  if (currentTimeTape + 1000 < millis()) {
    if (lineSensorValues[1] > 700) {
      //CHECKS MIDDLE LINE SENSOR FOR TAPE after 1 second
      currentTimeTape = millis();
      countTape++;
      tapeChange = false;
      encoders.getCountsAndResetLeft();
      encoders.getCountsAndResetRight();
      bip();

      display.clear();
      display.gotoXY(0, 1);
      display.print(countTape);
      //SKRIV DISPLAY PRINT IND HER, NÅR DU ER DONE
    }
  }
}

float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  return (countsL + countsR) / 2;
}

void bip() {
  buzzer.playNote(NOTE_A(4), 20, 15);
}

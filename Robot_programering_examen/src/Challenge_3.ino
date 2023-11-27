#include <Wire.h>
#include <Zumo32U4.h>
//Setting up liberays to use later in the code.
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;

int Stage = 0;

void setup() {
}


void loop() {

  if (buttonA.isPressed()) {
    Stage = 2;
  }
  switch (Stage) {
    case 1:
      motors.setSpeeds(0, 0);
      break;

    case 2:
      wallTurn();
      break;
  }
}


void wallTurn() {
#define NUM_SENSORS 6
  uint16_t lineSensorValues[NUM_SENSORS];
  int16_t Levels[10] = { 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };
  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();
  proxSensors.setBrightnessLevels(Levels, 10);
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
  proxSensors.read();
  display.clear();
  if (lineSensorValues[1] > 900) {
    Stage = 1;
  } else if (proxSensors.countsLeftWithLeftLeds() > proxSensors.countsRightWithRightLeds()) {
    if (proxSensors.countsLeftWithLeftLeds() || proxSensors.countsRightWithRightLeds() == 0) {
      if (proxSensors.countsFrontWithLeftLeds() > proxSensors.countsFrontWithRightLeds()) {
        motors.setSpeeds(350, 200);
      }
    } else {
      motors.setSpeeds(350, 200);
    }
  } else if (proxSensors.countsRightWithRightLeds() > proxSensors.countsLeftWithLeftLeds() || proxSensors.countsFrontWithRightLeds() > proxSensors.countsFrontWithLeftLeds()) {
    if (proxSensors.countsRightWithRightLeds() || proxSensors.countsLeftWithLeftLeds() == 0) {
      if (proxSensors.countsFrontWithRightLeds() > proxSensors.countsFrontWithLeftLeds()) {
        motors.setSpeeds(200, 350);
      }
    } else {
      motors.setSpeeds(200, 350);
    }
  } else {

    motors.setSpeeds(350, 350);
  }
}

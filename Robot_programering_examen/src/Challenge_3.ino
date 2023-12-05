#include <Wire.h>
#include <Zumo32U4.h>
//Setting up liberays to use later in the code.
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4LineSensors lineSensors;
Zumo32U4OLED display;

int Stage = 0;
uint16_t lineSensorValues[3];
uint16_t brightnessLevel[120];
uint8_t levelCount = 120;

void setup() {
  proxSensors.initThreeSensors();
  lineSensors.initThreeSensors();
  for (int i = 0; i <= 120; i++) {
    display.print("Setting up Sensors");
    brightnessLevel[i] = i;
  }
  proxSensors.setBrightnessLevels(brightnessLevel, levelCount);
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}


void loop() {
  proxSensors.read();

  switch (Stage) {
    case 0:
      display.clear();
      display.print("Press A");
      if (buttonA.isPressed()) {
        Stage = 2;
      }
      break;
    case 1:
      break;

    case 2:
      wallTurn();
      break;
  }
}

void wallTurn() {


  display.clear();
  proxSensors.read();

  if (lineSensorValues[1] > 900) {
    motors.setSpeeds(0, 0);
    display.clear();
    display.print("Stop");
    Stage = 1;
  } else if (proxSensors.countsLeftWithLeftLeds() > proxSensors.countsRightWithRightLeds()) {
    display.clear();
    display.print("Right");
    motors.setSpeeds(400, 200);
    if (proxSensors.countsLeftWithLeftLeds() == 0 || proxSensors.countsRightWithRightLeds() == 0) {
      if (proxSensors.countsFrontWithLeftLeds() > proxSensors.countsFrontWithRightLeds()) {
        motors.setSpeeds(400, 200);
      } else {
        display.clear();
        display.print("Forward");
        motors.setSpeeds(400, 400);
      }
    }
  } else if (proxSensors.countsRightWithRightLeds() > proxSensors.countsLeftWithLeftLeds()) {
    display.clear();
    display.print("Left");
    motors.setSpeeds(200, 400);
    if (proxSensors.countsRightWithRightLeds() == 0 || proxSensors.countsLeftWithLeftLeds() == 0) {
      if (proxSensors.countsFrontWithRightLeds() > proxSensors.countsFrontWithLeftLeds()) {
        motors.setSpeeds(200, 400);
      } else {
        display.clear();
        display.print("Forward");
        motors.setSpeeds(400, 400);
      }
    }
  } else {
    display.clear();
    display.print("Forward");
    motors.setSpeeds(400, 400);
  }
}

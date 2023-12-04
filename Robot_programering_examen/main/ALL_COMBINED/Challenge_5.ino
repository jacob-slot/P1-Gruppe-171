void Program5() {
  //SETUP

  display.clear();
  display.print(F("Press A"));
  buttonA.waitForButton();

  resetEncoders();
  display.clear();
  proxSensors.setPulseOffTimeUs(0.5);  //Proximity sensors pulse every 1ms. Reduces the delay in the code
  proxSensors.setPulseOnTimeUs(0.5);   // -----||------
  for (int i = 0; i < 120; i++) {
    //Makes array with values 0->119
    brightnessLevel[i] = i;
  }

  proxSensors.setBrightnessLevels(brightnessLevel, levelCount);  //Prox sensors gives back 120 levels

  display.gotoXY(2, 0);
  display.print("Step");
  display.gotoXY(2, 1);
  display.print("Away");

  delay(2000);
  display.clear();



  //LOOP
  while (true) {
    challengeFive();
  }
}


void turnRight() {
  //Controls the motors when going clockwise around pilar

  const int distanceToStraightRight = 2240;  //2250
  const int straighDistanceRight = 3400;
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

  int distanceToStraightLeft = 2290;  //2300
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
    tapeChange = false;
    encoders.getCountsAndResetLeft();
    encoders.getCountsAndResetRight();
    bip();

    display.clear();
    display.gotoXY(0, 0);
    display.print("Tape pass:");
    display.gotoXY(0, 1);
    display.print(countTape);
    //SKRIV DISPLAY PRINT IND HER, NÅR DU ER DONE
  }
}

float getDistance() {
  int countsL = encoders.getCountsLeft();
  int countsR = encoders.getCountsRight();
  return (countsL + countsR) / 2;
}

void bip() {
  buzzer.playNote(NOTE_A(4), 20, 15);
  delay(30);
}

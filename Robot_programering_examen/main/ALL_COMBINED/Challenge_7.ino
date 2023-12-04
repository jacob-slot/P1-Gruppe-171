void Program7() {
  
  //SETUP
  setup467();
  


  //LOOP
  while (true) {
    loop67();
  }
}

// Read linesensors
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void readEncoders() {
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

float radToDeg(float i) {
  return i * 180 / PI;
}

int mapLeft() {
  return map(lineSensorValues[0], lineSensorCal[0], lineSensorCalT[0], 0, 1000);
}

int mapMiddel() {
  return map(lineSensorValues[1], lineSensorCal[1], lineSensorCalT[1], 0, 1000);
}

int mapRight() {
  return map(lineSensorValues[2], lineSensorCal[2], lineSensorCalT[2], 0, 1000);
}

void updateMotorsToDriveStraight() {

  if (error < 0.0)  // if error < 0 zumo to far to the right
  {
    motors.setSpeeds(speedLeft, speedRight + abs(error) * 10);
  } else  // if error > 0 zumo to far to the left
  {
    motors.setSpeeds(speedLeft + abs(error) * 10, speedRight);
  }
}

int calGyro() {

  for (uint16_t i = 0; i < 3000; i++) {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  return total / 3000;
}

void turn() {
  if (accumulated_rotation_deg > angleLine && angleLine < 0) {
    motors.setSpeeds(-70, 70);
  }

  if (accumulated_rotation_deg < angleLine && angleLine > 0) {
    motors.setSpeeds(70, -70);
  }

  if (abs(accumulated_rotation_deg + angleLine) < 0.25) {
    motors.setSpeeds(0, 0);
    stage = 5;
    t = 0;
    display.clear();
    if (scrollValue == 5) {
      //If challenge 6 has been chosen, this will be run. 
      delay(1000);
      turnByAngle(currentCount);
    }
  }
}

void allign() {

  // Move forward
  motors.setSpeeds(70, 70);

  while (stage < 4) {

    // Read sensors
    readLineSensors();

    // Stage 1
    if (mapLeft() > value && stage == 1) {
      resetEncoders();
      stage = 2;
      side = 1;
      Serial.println(stage);
    }

    if (mapRight() > value && stage == 1) {
      resetEncoders();
      stage = 2;
      side = 2;
      Serial.println(stage);
    }

    // Stage 2
    if (side == 1 && mapRight() > 450 && stage == 2) {
      readEncoders();
      motors.setSpeeds(0, 0);
      stage = 3;
      Serial.println(stage);
    }

    if (side == 2 && mapLeft() > 500 && stage == 2) {
      readEncoders();
      motors.setSpeeds(0, 0);
      stage = 3;
      Serial.println(stage);
    }

    // Stage 3
    if (stage == 3 && side == 1) {

      distLine = ((countsLeft + countsRight) / 2.0) / 910.0 * 11.466;
      angleLine = radToDeg(atan(distLine / distSensor)) * (-1.0);
      stage = 4;
      display.clear();
      display.print(angleLine);
    }

    if (stage == 3 && side == 2) {

      distLine = ((countsLeft + countsRight) / 2.0) / 910.0 * 11.466;
      angleLine = radToDeg(atan(distLine / distSensor)) * 1.07;
      stage = 4;
      display.clear();
      display.print(angleLine);
    }
  }
}

void readGyro() {

  if (imu.gyroDataReady() == true) {
    // Read all IMU sensors: Gyro, Mag, Acc
    imu.read();

    // Calculate time since last messuremant
    time_now = micros();
    delta_time = (time_now - last_time);
    last_time = time_now;

    // imu.g.z returns a value in 0.07 deg / sek
    turn_rate = (imu.g.z + gyro_cal);

    rotation_deg = turn_rate * delta_time / 1000000.0 * 0.07;
    accumulated_rotation_deg += rotation_deg;
  }
}

void setup467() {
  // Init 3 linsensors
  lineSensors.initThreeSensors();

  // Clear display
  display.clear();

  // start wire to cumunicate with gyro
  Wire.begin();

  // Initialise Gyro
  imu.init();
  imu.configureForTurnSensing();
  delay(1000);

  display.print("Press A to");
  display.gotoXY(0, 1);
  display.print("calibrate");
  display.gotoXY(0, 2);
  display.print("Gyro and");
  display.gotoXY(0, 3);
  display.print("Board");




  buttonA.waitForPress();
  bip();

  buttonA.waitForRelease();  // wait till the button is released
  delay(500);

  // Print to Oled
  display.clear();
  display.print("Gyro Cal.");
  bip();

  // Cal gyro
  gyro_cal = calGyro() * (-1);

  // Calibrate line sensors for background
  display.clear();
  display.print("Board cal.");
  bip();

  int32_t lineTotals[3] = { 0, 0, 0 };

  for (uint16_t i = 0; i < 500; i++) {

    readLineSensors();
    lineTotals[0] += lineSensorValues[0];
    lineTotals[1] += lineSensorValues[1];
    lineTotals[2] += lineSensorValues[2];
    delay(5);
  }

  lineSensorCal[0] = lineTotals[0] / 500;
  lineSensorCal[1] = lineTotals[1] / 500;
  lineSensorCal[2] = lineTotals[2] / 500;

  // move to tape

  display.gotoXY(0, 0);
  display.clear();
  display.print("Move to");
  display.gotoXY(0, 1);
  display.print("tape and");
  display.gotoXY(0, 2);
  display.print("press a");




  buttonA.waitForPress();
  bip();
  buttonA.waitForRelease();  // wait till the button is released

  delay(1000);

  // Calibrate line sensors for tape values

  int32_t lineTotalsTape[3] = { 0, 0, 0 };

  for (uint16_t i = 0; i < 500; i++) {

    readLineSensors();
    lineTotalsTape[0] += lineSensorValues[0];
    lineTotalsTape[1] += lineSensorValues[1];
    lineTotalsTape[2] += lineSensorValues[2];
    delay(5);
  }

  lineSensorCalT[0] = lineTotalsTape[0] / 500;
  lineSensorCalT[1] = lineTotalsTape[1] / 500;
  lineSensorCalT[2] = lineTotalsTape[2] / 500;

  // Done
  bip();
  display.clear();
  display.gotoXY(0, 0);
  display.print("Done");
  display.gotoXY(0, 1);
  display.print("Press A");

  buttonA.waitForPress();
  bip();
  buttonA.waitForRelease();  // wait till the button is released
}

void loop67() {
  // Read sensors
  readLineSensors();

  // Get angle to line
  if (stage < 4) {
    allign();
  }

  // Turn parralel to line
  if (stage == 4) {
    if (t == 0) {
      start = millis();
      t = 1;
      accumulated_rotation_deg = 0;
    }
    readGyro();
    turn();
  }

  // drive streight
  if (stage == 5) {

    // Set angle to 0
    if (t == 0) {
      start = millis();
      t = 1;
      accumulated_rotation_deg = 0;
    }

    // Read the gyro
    readGyro();

    error = accumulated_rotation_deg;

    // Ajust motorspeeds to drive streight
    updateMotorsToDriveStraight();
  }

  if (stage == 5 && millis() - start > 500 && (mapMiddel() > 500 || mapLeft() > 500 || mapRight() > 500)) {
    // Stop program
    motors.setSpeeds(0, 0);
    stage = 10;
    display.clear();
    display.print("Press A");
  }


  if (buttonA.isPressed() == true) {
    stage = 1;
    t = 0;
    buttonA.waitForRelease();

    if (scrollValue == 5) {
      //If challenge 6 has been chosen, this will be run. 
      selectAngle();
    }
  }
}
void Program4() {
  //SETUP
  
  setup467();
  stage = 0;

  //LOOP
  while (true) {
    loopChallenge4();
  }
}

void selectMovement() {
  Serial.println("Select movment");
  
  readEncoders();
  if (countsRight > 5) {
    bip();
    resetEncoders();
    distance += 0.5;

    display.clear();
    display.gotoXY(0, 0);
    display.print("Cm.:");
    display.gotoXY(0, 1);
    display.print(distance);

  } else if (countsRight < -5) {
    bip();
    resetEncoders();
    distance += -0.5;
    if (distance < 0) distance = 0;
    display.clear();
    display.gotoXY(0, 0);
    display.print("Cm.:");
    display.gotoXY(0, 1);
    display.print(distance);
  }



  if (buttonA.isPressed()) {
    bip();
    stage = 1;
    distance += 2; //Adjusting for tape(2 cm)
    buttonA.waitForRelease();  // wait till the button is released
    delay(500);
  }
}

void loopChallenge4() {

  // Read sensors
  readLineSensors();

  Serial.println(stage);

  // Get distance from user input
  if (stage == 0) {
    selectMovement();
  }


  // Get angle to line
  if (0 < stage && stage < 4) {
    allign();
    Serial.println("After Select allign");
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

  // forward
  if (stage == 5) {
    motors.setSpeeds(80, 80);
    delay(300);
    stage = 6;
  }

  readLineSensors();

  // Reverse
  if (stage == 6) {

    // Set angle to 0
    if (t == 0) {
      start = millis();
      t = 1;
      accumulated_rotation_deg = 0;
    }

    // Reverse motors
    motors.setSpeeds(-80, -80);

    if ((mapMiddel() > 500 || mapLeft() > 500 || mapRight() > 500)) {
      motors.setSpeeds(0, 0);
      stage = 7;
      resetEncoders();
    }
  }

  // drive streight and distance
  if (stage == 7) {

    // Read the gyro
    readGyro();
    readEncoders();

    error = accumulated_rotation_deg;

    // Ajust motorspeeds to drive streight
    updateMotorsToDriveStraight();

    distanceEncoders = ((countsLeft + countsRight) / 2.0) / 910.0 * 11.466;

    if (distance < distanceEncoders) {
      stage = 10;
      motors.setSpeeds(0, 0);
    }
  }

  if (stage > 0) {
    display.gotoXY(0, 0);
    display.print("Distance:");
    display.gotoXY(0, 1);
    display.print(distanceEncoders+2); //Adjusting for tape(2 cm)

    
  }

  if (buttonA.isPressed() == true && stage > 2) {
    stage = 0;
    t = 0;
    distance = 0;
    buttonA.waitForRelease();
  }
}

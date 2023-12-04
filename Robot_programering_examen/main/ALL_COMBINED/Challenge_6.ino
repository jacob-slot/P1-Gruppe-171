void Program6() {
  //SETUP
  

  setup467();
  displayAngle();
  selectAngle();


  //LOOP
  while (true) {
    loop67();
  }
}

int turnByAngle(int movementParameter) {
  //This function will run in challenge 7-code if challenge 6 is chosen
  accumulated_rotation_deg = 0;
  motors.setSpeeds(100, 100);
  delay(500);

  if (movementParameter < 0) {
    motors.setSpeeds(100, -100);
    while (accumulated_rotation_deg > movementParameter) {
    
      readGyro();
    }
  } else {
    motors.setSpeeds(-100, 100);
    while (accumulated_rotation_deg < movementParameter) {
   
      readGyro();
    }
  }
}


void selectAngle() {
  //This function will run in challenge 7-code if challenge 6 is chosen
  readEncoders();
  currentCount = 0;

  while (buttonA.isPressed() == false) {
    readEncoders();
    if (countsRight < -5) {
      currentCount++;
      resetEncoders();
      displayAngle();


    } else if (countsRight > 5) {
      currentCount--;
      resetEncoders();
      displayAngle();
    }
  }
  delay(1000);
}

void displayAngle() {
  int printAngle = currentCount * (-1);  //Changes direction

  display.clear();
  display.gotoXY(0, 0);
  display.print("Angle:");
  display.gotoXY(0, 1);
  display.print(printAngle);
}
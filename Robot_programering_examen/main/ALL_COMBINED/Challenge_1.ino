void Program1() //WallStop
{
  //SETUP
  
  scrollValue = 0;
  displayChange("CmFromWall", "", "0cm", "");
  for(int i = 0; i < 120; i++)
  {
    brightnessLevels[i] = i;
  }
  proxSensors.setBrightnessLevels(brightnessLevels, 120);
  proxSensors.setPulseOffTimeUs(0);
  proxSensors.setPulseOnTimeUs(0);

  //LOOP
  while(true)
  {
    if (encoders.getCountsRight() > 50)
    {
      scrollValue = constrain(scrollValue + 1, 0, (round(countsToWall)/CPR)*TC);
      encoders.getCountsAndResetRight();
      itoa(scrollValue, text, 10);
      strncat(text, "cm", 9);
      displayChange("CmFromWall", "", text, "");
    }
    else if(encoders.getCountsRight() < -50)
    {
      scrollValue = constrain(scrollValue - 1, 0, (round(countsToWall)/CPR)*TC);
      encoders.getCountsAndResetRight();
      itoa(scrollValue, text, 10);
      strncat(text, "cm", 9);
      displayChange("CmFromWall", "", text, "");
    }

    if(buttonA.getSingleDebouncedRelease())
    {
      delay(1000);
      lineSensors.read(lineSensorValues);
      motors.setSpeeds(50,50);
      while(lineSensorValues[1] < 200) 
      {
        lineSensors.read(lineSensorValues);
      }
      
      motors.setSpeeds(0,0);
      lineSensors.emittersOff();
      encoders.getCountsAndResetRight();
      encoders.getCountsAndResetLeft();
      delay(1000);
      
      while(((encoders.getCountsRight() + encoders.getCountsLeft())/2) < (countsToWall-(scrollValue*CPR)/TC))
      {
        motors.setSpeeds(50,50);
        display.clear();
        display.gotoXY(0, 0);
        display.print(round((((encoders.getCountsRight() + encoders.getCountsLeft())/2)/CPR)*TC));
        display.gotoXY(0, 1);
        display.print(encoders.getCountsRight());
        display.gotoXY(0, 2);
        display.print(encoders.getCountsLeft());

        while(encoders.getCountsRight() > encoders.getCountsLeft())
        {
          motors.setSpeeds(75,50);
        }
        while(encoders.getCountsRight() < encoders.getCountsLeft())
        {
          motors.setSpeeds(50,75);
        }
      }

      motors.setSpeeds(0,0);
      encoders.getCountsAndResetRight();
      encoders.getCountsAndResetLeft();
      
      while(!buttonA.getSingleDebouncedRelease()){}
    }
  }
}  
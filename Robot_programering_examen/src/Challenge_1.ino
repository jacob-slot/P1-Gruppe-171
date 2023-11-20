#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4Encoders encoders;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxySensors;
Zumo32U4OLED display;
Zumo32U4ButtonA buttonA;

uint16_t brightnessLevels[120];
uint16_t lineSensorValues[3];
float CPR = 910; 
float TC = 11.466;
int16_t scrollValue = 0;
float countsToWall = 3360; //3359 or 3374
char text[9];

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

void setup() 
{
  Wire.begin();
  //Serial.begin(9600);
  encoders.init();
  lineSensors.initThreeSensors();
  proxySensors.initThreeSensors();
  for(int i = 0; i < 120; i++)
  {
    brightnessLevels[i] = (i+1);
  }
  proxySensors.setBrightnessLevels(brightnessLevels, 120);
  proxySensors.setPulseOffTimeUs(0);
  proxySensors.setPulseOnTimeUs(0);
  display.init();
  display.setLayout11x4();
  displayChange("PickProgram", programs[scrollValue], programs[scrollValue+1], programs[scrollValue+2]);
}

void loop() 
{
  if (encoders.getCountsRight() > 50)
  {
    scrollValue = constrain(scrollValue - 1, 0, listLength-2);
    encoders.getCountsAndResetRight();
    displayChange("PickProgram", programs[scrollValue], programs[scrollValue+1], programs[scrollValue+2]);
  }
  else if(encoders.getCountsRight() < -50)
  {
    scrollValue = constrain(scrollValue + 1, 0, listLength-2);
    encoders.getCountsAndResetRight();
    displayChange("PickProgram", programs[scrollValue], programs[scrollValue+1], programs[scrollValue+2]);
  }
  
  if(buttonA.getSingleDebouncedRelease())
  {
    encoders.getCountsAndResetRight();
    runProgram();
  }
}

void displayChange(char* Header, char* word1, char* word2, char* word3)
{
  display.clear();
  display.gotoXY(0,0);
  display.print(Header);
  display.gotoXY(0,1);
  display.print("^ ");
  display.print(word1);
  display.gotoXY(0,2);
  display.print("o ");
  display.print(word2);
  display.gotoXY(0,3);
  display.print("v ");
  display.print(word3);
}

void runProgram()
{
  switch(scrollValue)
  {
    case 0:
      Program1();
    break;
    case 1:
      Program2();
    break;
    case 2:
      Program3();
    break;
    case 3:
      Program4();
    break;
    case 4:
      Program5();
    break;
    case 5:
      Program6();
    break;
    case 6:
      Program7();
    break;
  }
}

void Program1() //WallStop
{
  scrollValue = 0;
  displayChange("CmFromWall", "", "0cm", "");
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

void Program2() //LineTest
{
  while(true)
  {
    lineSensors.read(lineSensorValues);
    display.clear();
    display.print(lineSensorValues[0]);
    display.gotoXY(0, 1);
    display.print(lineSensorValues[1]);
    display.gotoXY(0, 2);
    display.print(lineSensorValues[2]);
    delay(250);
  }
}

void Program3() //WallTurn
{
  while(true)
  {
    proxySensors.read();
    if(proxySensors.countsRightWithRightLeds() < 85 && proxySensors.countsLeftWithLeftLeds() < 85)
    {
    display.clear();
    display.gotoXY(0, 1);
    display.print("Stop");
    motors.setSpeeds(0,0);
    }
    else if(proxySensors.countsRightWithRightLeds() > proxySensors.countsLeftWithLeftLeds())
    {
    display.clear();
    display.gotoXY(0, 1);
    display.print("Left");
    motors.setSpeeds(0,50);
    }
    else if(proxySensors.countsLeftWithLeftLeds() > proxySensors.countsRightWithRightLeds())
    {
    display.clear();
    display.gotoXY(0, 1);
    display.print("Right");
    motors.setSpeeds(50,0);
    }
    else
    {
    display.clear();
    display.gotoXY(0, 1);
    display.print("Forward");
    motors.setSpeeds(50,50);
    }
    display.gotoXY(0, 2);
    display.print(proxySensors.countsLeftWithLeftLeds());
    display.gotoXY(0, 3);
    display.print(proxySensors.countsRightWithRightLeds());
  }
}

void Program4() //EncodersTest
{
  delay(500);
  motors.setSpeeds(50, 50);
  while((((encoders.getCountsRight() + encoders.getCountsLeft())/2)/CPR)*TC < 30.00)
  {
    display.clear();
    display.gotoXY(0, 0);
    display.print(round((((encoders.getCountsRight() + encoders.getCountsLeft())/2)/CPR)*TC));
    display.gotoXY(0, 1);
    display.print(encoders.getCountsRight());
    display.gotoXY(0, 2);
    display.print(encoders.getCountsLeft());
  }
  motors.setSpeeds(0, 0);
}

void Program5() //FrontProxyTest
{
  while(true)
  {
    display.clear();
    proxySensors.read();
    display.print(proxySensors.countsFrontWithLeftLeds());
    display.gotoXY(0, 1);
    display.print(proxySensors.countsFrontWithRightLeds());
    delay(250);
  }
}

void Program6() //SideProxyTest
{
  while(true)
  {
    display.clear();
    proxySensors.read();
    display.print(proxySensors.countsLeftWithLeftLeds());
    display.gotoXY(0, 1);
    display.print(proxySensors.countsRightWithRightLeds());
    delay(250);
  }
}

void Program7() //PrintTest
{
  //filler
}

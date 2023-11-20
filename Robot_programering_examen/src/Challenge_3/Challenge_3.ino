#include <Wire.h>
#include <Zumo32U4.h>
//Setting up liberays to use later in the code.
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;


void setup() 
{
  proxSensors.initThreeSensors();
  proxSensors.setBrightnessLevels(10, 50);
}


void loop() 
{
  wallTurn();
}
void wallTurn()
{
  proxSensors.read();

  if(proxSensors.countsRightWithRightLeds() > 40 && proxSensors.countsLeftWithLeftLeds() > 40)
  {
  motors.setSpeeds(0,0);
  }
  else if(proxSensors.countsRightWithRightLeds()>proxSensors.countsLeftWithLeftLeds())
  {
  motors.setSpeeds(15,150);
  }
  else if(proxSensors.countsLeftWithLeftLeds()>proxSensors.countsRightWithRightLeds())
  {
  motors.setSpeeds(160,0);
  }
  else
  {
  motors.setSpeeds(160,150);
  }
}

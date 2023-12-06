#include <Zumo32U4.h>
#include <Wire.h>

// Import from main lib
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED display;

uint16_t lineSensorValues[3];
int lineSensorCal[3];
int lineSensorCalT[3];

int stage = 1;
int side = 0;
float angleLine;
float distLine;

// Distance between linesensors
float distSensor = 9;

// Enconder counts for wheels
int16_t countsLeft;
int16_t countsRight;

// Values needed for keep track of change in time
int delta_time = 0;
int last_time = 0;
int time_now = 0;

// Turn rate
float turn_rate = 0;

// Rotation
float rotation_deg = 0;
float accumulated_rotation_deg = 0.0;

// Threshold value for indicating line being passed 
int value = 500;

// Gyro cal value
int gyro_cal = 0;
int32_t total = 0;

int32_t start = 0;
int t = 0;

int error = 0;

// Speeds for driving straight
int16_t speedLeft = 110;
int16_t speedRight = 110;

// Values for selecting menu value
bool selected = false;
int16_t scrollValue = 0;
float countsToWall = 3400;


float CPR = 910; 
float TC = 11.466;
char text[9];

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

// Read linesensors
void readLineSensors() 
{
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void readEncoders() 
{
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();
}

void resetEncoders() 
{
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

float radToDeg(float i) 
{
  return i * 180 / PI;
}

int mapLeft() 
{
  return map(lineSensorValues[0], lineSensorCal[0], lineSensorCalT[0], 0, 1000);
}

int mapMiddel() 
{
  return map(lineSensorValues[1], lineSensorCal[1], lineSensorCalT[1], 0, 1000);
}

int mapRight() 
{
  return map(lineSensorValues[2], lineSensorCal[2], lineSensorCalT[2], 0, 1000);
}

void updateMotorsToDriveStraight() 
{

  if (error < 0.0)  // if error < 0 zumo to far to the right
  {
    motors.setSpeeds(speedLeft, speedRight + abs(error) * 10);
  } 
  else  // if error > 0 zumo to far to the left
  {
    motors.setSpeeds(speedLeft + abs(error) * 10, speedRight);
  }
}

int calGyro() {

  for (uint16_t i = 0; i < 2000; i++) 
  {
    // Wait for new data to be available, then read it.
    while (!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  return total / 2000;
}

void turn() 
{
  if (accumulated_rotation_deg > angleLine && angleLine < 0) 
  {
    motors.setSpeeds(-70, 70);
  }

  if (accumulated_rotation_deg < angleLine && angleLine > 0) 
  {
    motors.setSpeeds(70, -70);
  }

  if (abs(accumulated_rotation_deg + angleLine) < 0.25) 
  {
    motors.setSpeeds(0, 0);
    stage = 5;
    t = 0;
  }
}

void allign() 
{
  // Move forward
  motors.setSpeeds(70, 70);

  while (stage < 4) 
  {

    // Read sensors
    readLineSensors();

    // Stage 1
    if (mapLeft() > value && stage == 1) 
    {
      resetEncoders();
      stage = 2;
      side = 1;
    }
    else if (mapRight() > value && stage == 1) 
    {
      resetEncoders();
      stage = 2;
      side = 2;
    }

    // Stage 2
    if (side == 1 && mapRight() > value && stage == 2) 
    {
      readEncoders();
      motors.setSpeeds(0, 0);
      stage = 3;
    }

    if (side == 2 && mapLeft() > value && stage == 2) 
    {
      readEncoders();
      motors.setSpeeds(0, 0);
      stage = 3;
    }

    // Stage 3
    if (stage == 3 && side == 1) 
    {
      distLine = ((countsLeft + countsRight) / 2.0) / CPR * TC;
      angleLine = radToDeg(atan(distLine / distSensor)) * (-1.0);
      stage = 4;
      display.clear();
      display.print(angleLine);
    }

    if (stage == 3 && side == 2) 
    {
      distLine = ((countsLeft + countsRight) / 2.0) / CPR * TC;
      angleLine = radToDeg(atan(distLine / distSensor)) * 1.07;
      stage = 4;
      display.clear();
      display.print(angleLine);
    }
  }
}

void readGyro() 
{
  if (imu.gyroDataReady() == true) 
  {
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


void setup() 
{
  // Init 3 linsensors
  lineSensors.initThreeSensors();

  // Init encoders
  encoders.init();

  // Init display
  display.init();
  display.setLayout11x4();

  // Start wire to cumunicate with gyro
  Wire.begin();

  // Initialise Gyro
  imu.init();
  imu.configureForTurnSensing();

  // Print to Oled
  display.print("Gyro Cal");
  delay(500);

  // Cal gyro
  gyro_cal = calGyro() * (-1);

  // Calibrate line sensors for background
  display.clear();
  display.print("Floor");

  int32_t lineTotals[3] = { 0, 0, 0 };

  for (uint16_t i = 0; i < 500; i++) 
  {
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
  display.clear();
  display.print("M to Tape");

  buttonA.waitForPress();
  buttonA.waitForRelease();  // wait till the button is released

  delay(500);

  // Calibrate line sensors for tape values

  int32_t lineTotalsTape[3] = { 0, 0, 0 };

  for (uint16_t i = 0; i < 500; i++) 
  {
    readLineSensors();
    lineTotalsTape[0] += lineSensorValues[0];
    lineTotalsTape[1] += lineSensorValues[1];
    lineTotalsTape[2] += lineSensorValues[2];
    delay(5);
  }

  lineSensorCalT[0] = lineTotalsTape[0] / 500;
  lineSensorCalT[1] = lineTotalsTape[1] / 500;
  lineSensorCalT[2] = lineTotalsTape[2] / 500;

  displayChange("CmFromWall", "", "0cm", "");
}

void loop() 
{
  scrollValue = 0;
  while (!selected)
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
      selected = true;
      delay(1000);
    }
  }

  // Get angle to line
  if (stage < 4) 
  {
    allign();
  }
 
  // Turn parralel to line
  while (stage == 4) 
  {
    if (t == 0) 
    {
      start = millis();
      t = 1;
      accumulated_rotation_deg = 0;
    }
    readGyro();
    turn();
  }

  // drive straight
  while(stage == 5) 
  {
    // Set angle to 0
    if (t == 0) 
    {
      start = millis();
      t = 1;
      accumulated_rotation_deg = 0;
    }

    // Read the gyro
    readGyro();

    error = accumulated_rotation_deg;

    // Ajust motorspeeds to drive straight
    updateMotorsToDriveStraight();

    if (((encoders.getCountsRight() + encoders.getCountsLeft())/2) > (countsToWall-(scrollValue*CPR)/TC))
    {
      motors.setSpeeds(0,0);
      stage = 1;
      t = 0;
      selected = false;
      resetEncoders();
      displayChange("CmFromWall", "", "0cm", "");
    }
  }
}

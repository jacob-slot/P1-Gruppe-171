#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED lcd;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;

int const countJump = 100;
int movementTurn = 0;   // used to select the degree turned
int const maxAngle = 359; // max angle that can be turned
int speed = 100; // max speed


// Gyro variables
uint32_t turnAngle = 0;
int16_t turnRate;
int16_t gyroOffset;
uint16_t gyroLastUpdate = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  turnSensorSetup();
  delay(100);
}

//Resets encoders
void resetEncoders(){
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}
//Read the encoder to get the degree
void readEncodersDegree(){
  movementTurn = movementTurn + encoders.getCountsRight();
  resetEncoders();
}
//Function to print in the display
void printLCD(String s0, String s1){
  lcd.clear();
  lcd.print(s0);
  lcd.gotoXY(0,1);
  lcd.print(s1);
}

void LCDSelectDegree(){
  printLCD("Angle", (String)movementTurn);
}

void turnByAngle(){
  turnSensorReset();
  selectAngle();
  if (buttonA.isPressed()){
    delay(100);
  if (movementTurn>180){
    motors.setSpeeds(100,-100);
    while(getTurnAngleInDegrees()>movementTurn){
      delay(10);
    }
  }
  else{
    motors.setSpeeds(-100,100);
    while(getTurnAngleInDegrees()<movementTurn){
      delay(10);
    }
  }
  }
}

void turnSensorSetup()
{
  Wire.begin();
  imu.init();
  imu.enableDefault();
  imu.configureForTurnSensing();

  lcd.clear();
  lcd.print(F("Gyro cal"));

  delay(500);

  // Calibrate the gyro.
  int32_t total = 0;
  for (uint16_t i = 0; i < 1024; i++)
  {
    // Wait for new data to be available, then read it.
    while(!imu.gyroDataReady()) {}
    imu.readGyro();

    // Add the Z axis reading to the total.
    total += imu.g.z;
  }
  ledYellow(0);
  gyroOffset = total / 1024;


  lcd.clear();
  turnSensorReset();
}

void turnSensorReset()
{
  gyroLastUpdate = micros();
  turnAngle = 0;
}

void turnSensorUpdate()
{
  imu.readGyro();
  turnRate = imu.g.z - gyroOffset;

  uint16_t m = micros();
  uint16_t dt = m - gyroLastUpdate;
  gyroLastUpdate = m;

  int32_t d = (int32_t)turnRate * dt;

  turnAngle += (int64_t)d * 14680064 / 17578125;
}

uint32_t getTurnAngleInDegrees(){
  turnSensorUpdate();
  return (((uint32_t)turnAngle >> 16) * 360) >> 16;
}

void selectAngle(){
    readEncodersDegree();
  if (movementTurn < 0){
    movementTurn = 0;
  }
  else if (movementTurn > maxAngle){
    movementTurn = maxAngle;
  }
  LCDSelectDegree();
}

void stop(){
  motors.setSpeeds(0,0);
}

void loop() {
  // put your main code here, to run repeatedly:
  turnByAngle();
  stop();

}

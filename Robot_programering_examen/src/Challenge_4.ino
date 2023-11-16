#include <Zumo32U4.h>
#include <Wire.h>

// import from main lib
Zumo32U4LineSensors lineSensors;
Zumo32U4ButtonA buttonA;
Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4IMU imu;
Zumo32U4OLED lcd;
Zumo32U4Buzzer buzzer;

#define NUM_SENSORS 3
int16_t lineSensorValues[NUM_SENSORS];

int lineSensorCal[3];
int lineSensorCalT[3];

int stage = 0;
int side = 0;
float angleLine;
float distLine;

float distance = 0;
float distanceEncoders = 0;

// Distance between linesensors
float distSensor = 9;

// Enconder counts for wheels
int16_t countsLeft;
int16_t countsRight;

// Stuff needed for delta time
int delta_time = 0;
int last_time = 0;
int time_now = 0;

// Turn rate
float turn_rate = 0;

// Rotation
float rotation_deg = 0;
float accumulated_rotation_deg = 0.0;

// threshold line
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

// Read linesensors
void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void bip() {
  buzzer.playNote(NOTE_A(4), 20, 15);
  delay(30);
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

void selectMovement() {
  Serial.println("Select movment");
  readEncoders();
  if (countsRight > 5) {
    bip();
    resetEncoders();
    distance += 0.5;
  } else if (countsRight < -5) {
    bip();
    resetEncoders();
    distance += -0.5;
    if (distance < 0) distance = 0;
  }

  lcd.clear();
  lcd.print(distance);

  if (buttonA.isPressed()) {
    bip();
    stage = 1;
    distance += -2.2;
    buttonA.waitForRelease();  // wait till the button is released
    delay(500);
  }
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
      lcd.clear();
      lcd.print(angleLine);
    }

    if (stage == 3 && side == 2) {

      distLine = ((countsLeft + countsRight) / 2.0) / 910.0 * 11.466;
      angleLine = radToDeg(atan(distLine / distSensor)) * 1.07;
      stage = 4;
      lcd.clear();
      lcd.print(angleLine);
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


void setup() {

  // Init 3 linsensors
  lineSensors.initThreeSensors();

  //start serial 9600
  Serial.begin(9600);

  // Clear display
  lcd.clear();

  // start wire to cumunicate with gyro
  Wire.begin();

  // Initialise Gyro
  imu.init();
  imu.configureForTurnSensing();
  delay(1000);

  // Print to Oled
  lcd.print("Gyro");
  Serial.println("Do not move Zumo, gyro calibration");
  delay(1000);

  // Cal gyro
  gyro_cal = calGyro() * (-1);

  // Calibrate line sensors for background
  lcd.clear();
  lcd.print("Floor");

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
  lcd.clear();
  lcd.print("Tape");

  Serial.println("Move to tape and press A");
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

  lcd.clear();
  lcd.print("Done");
  bip();
  delay(500);
}

void loop() {

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
   if (stage == 5){
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
    lcd.gotoXY(0, 1);
    lcd.print(accumulated_rotation_deg);
    lcd.gotoXY(0, 0);
  }

  if (buttonA.isPressed() == true && stage > 2) {
    stage = 0;
    t = 0;
    distance = 0;
    buttonA.waitForRelease();
  }
}

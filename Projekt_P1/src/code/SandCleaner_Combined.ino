#include <Zumo32U4.h>
#include "FastIMU.h"
#include <Wire.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4LineSensors lineSensors;

//---------------------------------------------------------------------
// Dead Reconing

// Define motor constants
#define WHEEL_RADIUS 1.801075         // Replace with your actual wheel radius in centimeter
#define WHEEL_DISTANCE 8.500          // Replace with your actual wheel distance in centimeters
#define ENCODER_STEPS_PER_REV 910.00  // 910 steps per wheel revolution

// Variables for storing encoder counts
long leftEncoderCount = 0;
long rightEncoderCount = 0;

// Variables for storing previous encoder counts
long prevLeftEncoderCount = 0;
long prevRightEncoderCount = 0;

// Variables for storing robot position
double x = 0.0;      // X position in meters
double y = 0.0;      // Y position in meters
double theta = 0.0;  // Orientation in radians
double theta_1 = 0.0;

// Variable for storing the last time
unsigned long lastTimeMicros = 0;

//--------------------------------------------------------------------------------------------------
// Linesensor stuff
#define NUM_SENSORS 3
uint16_t lineSensorValues[NUM_SENSORS];

//--------------------------------------------------------------------------------------------------
// Gyro stuff
#define IMU_6500 0x68        // Address of the IMU
#define IMU_LSM6DS3 0x6B     // Address of the IMU
#define PERFORM_CALIBRATION  //Comment to disable startup calibration
MPU6500 IMU_1;               //Change to the name of any supported IMU!
LSM6DS3 IMU_2;

calData calib = { 0 };    //Calibration data MPU6500
calData calib_2 = { 0 };  //Calibration data LSM6DS3

AccelData accelData;  //Sensor data
AccelData accelData_2;
GyroData gyroData;
GyroData gyroData_2;

//---------------------------------------------------------------------------------------------------
// All variables

bool finalLine = false;

bool setDelta = false;

bool setPrint = false;

double distNow = 0;

double encoderSum = 0;

// Line sensor tape threshold
int tape = 700;

// Switch case stage
int stage = 1;

// Drive straight variables
int speed = 110;

// Threshold for imu sensor noize
float threshold = 0.15;

// t used to set time at void loop startup
int t = 0;
int32_t time_start = 0;
double time_print = 0;

// Stuff needed for delta time sinse last messurement
int32_t delta_time = 0;
int32_t last_time = 0;
int32_t time_now = 0;

// Rotation IMU_6500
float rotation_deg_6500 = 0;
float accumulated_rotation_deg_6500 = 0.0;

// Rotation IMU_LSM6DS3
float rotation_deg_lsm = 0;
float accumulated_rotation_deg_lsm = 0.0;

// IMU6500 and LSM6DS3 combined
float combined = 0;
float rotation_deg = 0;
float accumulated_rotation_deg = 0.0;

//VARIABLES FOR OBSTACLE AVOIDANCE
const int levelCount = 15;
uint16_t brightnessLevel[levelCount];
bool closeToBox = false;
bool hittingBox = false;
bool runOnce = 0;
int obstacleStage = 1;
int currentGyro;
float timeNow;
int extraLength = 10;
float localAngle;
double obstacleEncoderSum;
int lastStage;


double toDeg(double radians) {
  return radians * 180.0 / PI;
}

double toRad(double degrees) {
  return degrees * (PI / 180);
}

void readLineSensors() {
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON);
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void readEncoders() {
  leftEncoderCount = encoders.getCountsLeft();
  rightEncoderCount = encoders.getCountsRight();
}

float encoderDistance() {
  return ((leftEncoderCount + rightEncoderCount) / 2.0) / 910.0 * 11.466;
}

void updatePosition() {

  // Read encoder values
  readEncoders();
  resetEncoders();

  // Get the current time in microseconds
  unsigned long currentTimeMicros = micros();

  // Calculate the elapsed time since the last update in microseconds
  unsigned long deltaTimeMicros = currentTimeMicros - lastTimeMicros;
  lastTimeMicros = currentTimeMicros;

  // delta is equal to EncoderCount because we call resetEncoders in updatePosition()
  long deltaLeft = leftEncoderCount;
  long deltaRight = rightEncoderCount;

  // Convert encoder counts to angular velocities (Unit) = radians/seconds
  double wLeft = (deltaLeft * 360.0 / ENCODER_STEPS_PER_REV) / deltaTimeMicros * PI / 180.0 * 1000000;
  double wRight = (deltaRight * 360.0 / ENCODER_STEPS_PER_REV) / deltaTimeMicros * PI / 180.0 * 1000000;

  // Calculate linear and angular velocities using the kinematic model
  double Vl = WHEEL_RADIUS * wLeft;
  double Vr = WHEEL_RADIUS * wRight;
  double V = (Vl + Vr) / 2;

  double omega = (Vl - Vr) / WHEEL_DISTANCE * (-1);  // reverse angle idk why

  // Integrate velocities to get changes in position
  double deltaTime = deltaTimeMicros;  // Convert microseconds to seconds

  theta_1 += omega * deltaTime / 1000000.00000;
  theta = toRad(accumulated_rotation_deg_6500);

  x += V * cos(theta) * deltaTime / 1000000.00000;
  y += V * sin(theta) * deltaTime / 1000000.00000;
}

void updateMotorsToDriveStraight(float heading) {

  if (accumulated_rotation_deg_6500 < heading * (-1)) {
    motors.setSpeeds(speed, speed + abs(accumulated_rotation_deg_6500 + heading) * 20);
  } else {
    motors.setSpeeds(speed + abs(accumulated_rotation_deg_6500 + heading) * 20, speed);
  }

  if (lineSensorValues[1] > tape) {
    motors.setSpeeds(0, 0);
    stage++;

    if (finalLine == true) {
      stage = 30;
      obstacleStage = 30;
    }
  }
}

void turn(float angle) {
  if (accumulated_rotation_deg_6500 > angle * (-1)) {
    motors.setSpeeds(90, -90);
  }

  if (accumulated_rotation_deg_6500 < angle * (-1)) {
    motors.setSpeeds(-90, 90);
  }

  if (abs(accumulated_rotation_deg_6500 + angle) < 0.5) {
    motors.setSpeeds(0, 0);
    stage++;
  }
}

void Gyro() {

  // Calculate time since last messuremant
  time_now = micros() - time_start;
  delta_time = (time_now - last_time);
  last_time = time_now;

  // IMU6500
  rotation_deg_6500 = gyroData.gyroZ * delta_time / 1000000.0;
  if (abs(gyroData.gyroZ) > threshold) {
    accumulated_rotation_deg_6500 += rotation_deg_6500;
  }

  // LSM6DS3
  rotation_deg_lsm = gyroData_2.gyroZ * delta_time / 1000000.0;
  if (abs(gyroData_2.gyroZ) > threshold) {
    accumulated_rotation_deg_lsm += rotation_deg_lsm;
  }


  // LSM6DS3 and IMU6500
  rotation_deg = combined * delta_time / 1000000.0;
  accumulated_rotation_deg += rotation_deg;
}

void printToOpenlog() {

  if (setPrint == false) {
    setPrint = true;

    Serial1.print("Time [s]");
    Serial1.print("; \t");
    Serial1.print("Delta [ms]");
    Serial1.print("; \t");
    Serial1.print("Gyro [deg]");
    Serial1.print("; \t");
    Serial1.print("KIN [deg]");
    Serial1.print("; \t");
    Serial1.print("x [cm]");
    Serial1.print("; \t");
    Serial1.println("y [cm]");
  }


  Serial1.print(time_print);
  Serial1.print("; \t");
  Serial1.print(delta_time / 1000);
  Serial1.print("; \t");
  Serial1.print(accumulated_rotation_deg_6500);
  Serial1.print("; \t");
  Serial1.print(toDeg(theta_1));
  Serial1.print("; \t");
  Serial1.print(x);
  Serial1.print("; \t");
  Serial1.println(y);
}

void printToSerial() {
  Serial.print(encoderSum);
  Serial.print("; ");
  Serial.print(delta_time / 1000.0);
  Serial.print("; ");
  Serial.print(time_print);
  Serial.print("; ");
  Serial.print(accumulated_rotation_deg_6500);

  Serial.print("\t X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Angle: ");
  Serial.println(toDeg(theta));
}

void alwaysOn() {

  // Everything in this if statment only happens once; first time the voil loop is entered
  if (t == 0) {
    time_start = micros();
    t = 1;
    accumulated_rotation_deg = 0.0;
    accumulated_rotation_deg_6500 = 0.0;
    accumulated_rotation_deg_lsm = 0.0;
    resetEncoders();
    Serial.println("T Set");
  }

  // This restarts the driving sequence
  if (stage == 9) { stage = 1; }

  //----------------------------------------------------------------------------------------------
  // All functions that need to run every loop

  // IMU Stuff
  IMU_1.update();
  IMU_2.update();
  IMU_1.getGyro(&gyroData);
  IMU_2.getGyro(&gyroData_2);
  Gyro();

  // read line sensors
  readLineSensors();

  // Update zumo position
  updatePosition();

  // Print to OpenLog and serial
  printToOpenlog();
  printToSerial();

  // variables that need to be updated
  combined = (gyroData_2.gyroZ + gyroData.gyroZ) / 2;
  encoderSum = encoderSum + encoderDistance();
  time_print = (micros() - time_start) / 1000000.0;

  //Sensorreading
  proxSensors.read();
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);  //400khz clock
  Serial.begin(115200);
  Serial1.begin(115200);
  lineSensors.initThreeSensors();  // initialize linesensors
                                   //PROX SETUP
  proxSensors.initThreeSensors();
  proxSensors.setPulseOffTimeUs(0);  //Proximity sensors pulse every 1ms. Reduces the delay in the code(Gyro can then run)
  proxSensors.setPulseOnTimeUs(0);   // -----||------
  for (int i = 0; i <= levelCount; i++) {
    //Makes array with values 0->levelCount
    brightnessLevel[i] = i;
  }
  proxSensors.setBrightnessLevels(brightnessLevel, levelCount);

  delay(2000);

  // Init IMU6500
  int err_IMU6500 = IMU_1.init(calib, IMU_6500);
  if (err_IMU6500 != 0) {
    Serial.print("Error initializing IMU6500: ");
    Serial.println(err_IMU6500);
    while (true) {
      ;
    }
  }

  // Init LSM6DS3
  int err_LSM6DS3 = IMU_2.init(calib, IMU_LSM6DS3);
  if (err_LSM6DS3 != 0) {
    Serial.print("Error initializing LSM6DS3: ");
    Serial.println(err_LSM6DS3);
    while (true) {
      ;
    }
  }

#ifdef PERFORM_CALIBRATION
  Serial.println("IMU calibration & data example for IMU6500 and LSM6DS3");

  delay(1000);
  Serial.println("Keep IMU level.");
  delay(1000);
  IMU_1.calibrateAccelGyro(&calib);
  IMU_2.calibrateAccelGyro(&calib_2);


  Serial.println("Calibration done!");

  // IMU6500 Ofsets
  Serial.println("Accel biases for IMU6500 X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);

  Serial.println("Gyro biases for IMU6500 X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);

  // LSM6DS3 Ofsets
  Serial.println("Accel biases for LSM6DS3 X/Y/Z: ");
  Serial.print(calib_2.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib_2.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib_2.accelBias[2]);

  Serial.println("Gyro biases for LSM6DS3 X/Y/Z: ");
  Serial.print(calib_2.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib_2.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib_2.gyroBias[2]);

  delay(1000);
  IMU_1.init(calib, IMU_6500);
  IMU_2.init(calib_2, IMU_LSM6DS3);

#endif
}


void loop() {

  // Do not DELETE!!
  alwaysOn();

  //--------------------------------------------------------------------------
  // Tobias Code here
  obstacleAvoidance();

  //--------------------------------------------------------------------------
  // Main driving program

  switch (stage) {
    case 1:  // Drive forward
      updateMotorsToDriveStraight(0.0);
      break;
    case 2:  // Turn Right 90 degrees
      turn(90.0);
      break;
    case 3:  // Drive
      updateMotorsToDriveStraight_1(90.0, 10);
      break;
    case 4:  // Turn
      turn(180.0);
      break;
    case 5:  // Drive forward
      updateMotorsToDriveStraight(180.0);
      break;
    case 6:  // Turn Right 180 degrees
      turn(90.0);
      break;
    case 7:  // Drive forward
      updateMotorsToDriveStraight_1(90.0, 10);
      break;
    case 8:  // Drive forward
      turn(0.0);
      break;
  }

  // Check if it is last line
  if (((stage == 3) || (stage == 7)) && (lineSensorValues[1] > tape)) {
    stage++;
    finalLine = true;
  }
}


void updateMotorsToDriveStraight_1(float heading, int distance) {

  if (!setDelta) {
    setDelta = true;
    distNow = encoderSum;
  }

  readEncoders();

  if (accumulated_rotation_deg_6500 < heading * (-1)) {
    motors.setSpeeds(speed, speed + abs(accumulated_rotation_deg_6500 + heading) * 20);
  } else {
    motors.setSpeeds(speed + abs(accumulated_rotation_deg_6500 + heading) * 20, speed);
  }

  if (encoderSum - distNow > distance) {
    motors.setSpeeds(0, 0);
    setDelta = false;
    if (obstacleStage > 2) {
      obstacleStage++;
    } else {
      stage++;
    }
  }
}




void obstacleTurn(float angle) {
  //ÆNDRET AF TOBIAS

  if (runOnce == 0) {
    localAngle = angle + accumulated_rotation_deg_6500;
    runOnce = 1;
  }


  if (accumulated_rotation_deg_6500 > localAngle) {
    motors.setSpeeds(80, -80);
  }

  if (accumulated_rotation_deg_6500 < localAngle) {
    motors.setSpeeds(-80, 80);
  }

  if (abs(accumulated_rotation_deg_6500 - localAngle) < 0.3) {
    motors.setSpeeds(0, 0);
    obstacleStage++;
    runOnce = 0;
    currentGyro = accumulated_rotation_deg_6500 * (-1);
  }
}

void drivingStraightObstacle(float heading) {
  //ÆNDRET AF TOBIAS
  if (accumulated_rotation_deg_6500 < heading * (-1)) {
    motors.setSpeeds(speed, speed + abs(accumulated_rotation_deg_6500 + heading) * 20);
  } else {
    motors.setSpeeds(speed + abs(accumulated_rotation_deg_6500 + heading) * 20, speed);
  }
}



void obstacleAvoidance() {
  switch (obstacleStage) {
    case 1:
      //Looking for obstacle

      if ((proxSensors.countsFrontWithRightLeds() > 9) || (proxSensors.countsFrontWithLeftLeds() > 9)) {
        //SPOTS OBSTACLE IN THE DISTANCE
        obstacleStage++;
      }
      break;
    case 2:
      //Obstacle detected, looking for impact
      if ((proxSensors.countsFrontWithRightLeds() < 7) || (proxSensors.countsFrontWithLeftLeds() < 7)) {
        //GETS VERY CLOSE TO OBSTACLE
        obstacleStage++;
        lastStage = stage;
        stage = 30;  //Stop Other switch case from running
        timeNow = millis();
      }
      break;

    case 3:
      //impact detected, back off  (Break out of jacobs code)
      motors.setSpeeds(-70, -70);
      if ((timeNow + 500) <= millis()) {
        obstacleStage++;
      }
      break;
    case 4:
      // Turn right
      obstacleTurn(-90.0);
      break;
    case 5:
      // Remember distance driven
      obstacleEncoderSum = encoderSum;
      obstacleStage++;
      break;
    case 6:
      //Drive around obstacle.
      followleft();
      Serial.println("I in case 5");
      break;
    case 7:
      //Drive small extra lengt
      updateMotorsToDriveStraight_1(currentGyro, extraLength);
      Serial.println("I in case 6");
      break;
    case 8:
      // Remember distance driven
      obstacleEncoderSum = encoderSum - obstacleEncoderSum;
      obstacleStage++;
      break;

    case 9:
      //no more obstacle, turn left
      obstacleTurn(90.0);
      Serial.println("I in case 7");
      break;
    case 10:
      //Drive small extra lengt
      updateMotorsToDriveStraight_1(currentGyro, extraLength);
      break;
    case 11:
      //Drive around obstacle.
      followleft();
      break;
    case 12:
      //Drive small extra lengt
      updateMotorsToDriveStraight_1(currentGyro, extraLength);
      break;
    case 13:
      //no more obstacle, turn left
      obstacleTurn(90.0);
      break;
    case 14:
      //drive (Same distance as case 5)
      updateMotorsToDriveStraight_1(currentGyro, obstacleEncoderSum);
      break;
    case 15:
      //Distance driven. Turn right.
      obstacleTurn(-90.0);
      break;
    case 16:
      //Go back to Jacobs code (Set code back to case 1)
      obstacleStage = 1;
      stage = lastStage;
  }
}

void followleft() {
  const int threshold = 0;
  drivingStraightObstacle(currentGyro);

  if (proxSensors.countsLeftWithLeftLeds() == threshold) {
    obstacleStage++;
  }
}

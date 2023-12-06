#include <Zumo32U4.h>

Zumo32U4Motors motors;
Zumo32U4Encoders encoders;
Zumo32U4OLED display;

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

// Variable for storing the last time
unsigned long lastTimeMicros = 0;

//----------------------------------------------------------------------

double toDeg(double radians) {
  return radians * 180.0 / PI;
}

// Function to update zumo position based on encoder readings
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

  double omega = (Vl - Vr) / WHEEL_DISTANCE;

  // Integrate velocities to get changes in position
  double deltaTime = deltaTimeMicros;  // Convert microseconds to seconds
  theta += omega * deltaTime / 1000000.00000;
  x += V * cos(theta) * deltaTime / 1000000.00000;
  y += V * sin(theta) * deltaTime / 1000000.00000;

  /*
  // Debug prints

  Serial.print("wLeft: ");
  Serial.print(wLeft);
  Serial.print(" wRight: ");
  Serial.print(wRight);
  Serial.print(" Omega: ");
  Serial.print(omega);
  Serial.print(" Theta: ");
  Serial.print(theta);
  Serial.print(" V cm/s: ");
  Serial.print(V * 100);
  Serial.print(" cos: ");
  Serial.print(cos(theta));
  Serial.print(" sin: ");
  Serial.print(sin(theta));
  Serial.print(" Left: ");
  Serial.print(deltaLeft);
  Serial.print(" Right: ");
  Serial.print(deltaRight);
  Serial.print(" D time: ");
  Serial.print(deltaTime);
  Serial.print("\t");
  */
}

void resetEncoders() {
  encoders.getCountsAndResetLeft();
  encoders.getCountsAndResetRight();
}

void readEncoders() {
  leftEncoderCount = encoders.getCountsLeft();
  rightEncoderCount = encoders.getCountsRight();
}


void setup() {

  Serial.begin(115200);
  display.init();
  delay(2000);
}

void loop() {

  updatePosition();
  motors.setSpeeds(80, 80);

  delay(10);
  display.clear();

  display.gotoXY(0, 0);
  display.print(x);
  display.gotoXY(0, 1);
  display.print(y);
  display.gotoXY(0, 2);
  display.print(toDeg(theta));


  // Print current position (for testing purposes)
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Angle: ");
  Serial.println(toDeg(theta));
}

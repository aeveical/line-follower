// define pins for motors and sensors
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 3
#define IR_RIGHT_IN A1
#define IR_LEFT_IN A2

// import the motor shield library to control motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// initalize the motershield library to run commands through
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// assign a motor "object" for the right and left motors
Adafruit_DCMotor *motorRight = AFMS.getMotor(MOTOR_RIGHT);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(MOTOR_LEFT);

// define variables that will control motor speed
float motorRightSpeed = 0;
float motorLeftSpeed = 0;
// define the initial values that the sensor reads (when it is just on the carpet)
int IRSensorRight_Initial = 0;
int IRSensorLeft_Initial = 0;
// define variables that store the value the sensor is reading out
int IRSensorRight = 0;
int IRSensorLeft = 0;
// define variables that store the difference between the initial sensor values and the current reading
int sensorDifferenceRight = 0;
int sensorDifferenceLeft = 0;
// set difference threshold
const int threshold = 50;
// set initial direction of wheels (one wheel had switched wiring, so its default to go forward is backward in code)
int rightMotorDirection = BACKWARD;
int leftMotorDirection = FORWARD;
// set base speed of wheels
const float BASE = 25;
// set wheel ratio (how fast the wheels go when turning)
float wheelRatio = 3.5;

void setup() {
  // initalize the serial port
  Serial.begin(9600);
  // set the ir sensor pins to be input
  pinMode(IR_RIGHT_IN, INPUT);
  pinMode(IR_LEFT_IN, INPUT);

  // read the sensor values one at setup (assumes sensors are on default ground)
  IRSensorRight_Initial = analogRead(IR_RIGHT_IN);
  IRSensorLeft_Initial = analogRead(IR_LEFT_IN);

  // start the motor library
  AFMS.begin();
}

void loop() {

  // serial control for port
  if (Serial.available() > 0) {
    // if something has been sent through the serial port, assume it's
    // a new wheel ratio and set the wheel ratio as such
    String word = Serial.readString()
    wheelRatio = word;
  }

  // read the sensor values
  IRSensorRight = analogRead(IR_RIGHT_IN);
  IRSensorLeft = analogRead(IR_LEFT_IN);

  // print values of sensors live to use in plot
  Serial.println("IR RIGHT:")
  Serial.println(IRSensorRight);
  Serial.println("IR LEFT:")
  Serial.println(IRSensorLeft);

  // calculate the difference in initial and current sensor values
  sensorDifferenceRight = IRSensorRight_Initial - IRSensorRight;
  sensorDifferenceLeft = IRSensorLeft_Initial - IRSensorLeft;

  // reset the motor speeds to be running at base speed
  motorRightSpeed = BASE;
  motorLeftSpeed = BASE;
  // reset the motor directions to be running forward
  rightMotorDirection = BACKWARD;
  leftMotorDirection = FORWARD;
  // move the motors in the default direction and speed
  Move(motorRightSpeed, motorLeftSpeed, rightMotorDirection, leftMotorDirection);

  if (sensorDifferenceRight > threshold) {
    // if the sensor detects a change in enviornment (i.e it goes over the tape),
    // the course needs to recorrect by increasing the speed of the right wheel and
    // decrease the speed of the left
    
    // turn left
    motorLeftSpeed = BASE*wheelRatio;
    leftMotorDirection = BACKWARD;
    motorRightSpeed = BASE*wheelRatio;
  } else if (sensorDifferenceLeft > threshold) {
    // same conditions but for turning in the other direction

    // turn right
    motorRightSpeed = BASE*wheelRatio;
    rightMotorDirection = FORWARD;
    motorLeftSpeed = BASE*wheelRatio;
  }

  // once the speed/direction of the wheels is decided, move the motors
  Move(motorRightSpeed, motorLeftSpeed, rightMotorDirection, leftMotorDirection);
  delay(10);

  // display motor speeds sent to arduino
  // for graphing purposes if the wheel was spinning backwards I represented it as a negative
  if (rightMotorDirection == FORWARD) {
    motorRightSpeed = -(motorRightSpeed)
  } else if (leftMotorDirection == BACKWARD) {
    motorLeftSpeed = -(motorLeftSpeed)
  }
  
  Serial.println("Right Motor Speeds:");
  Serial.println(motorRightSpeed);
  Serial.println("Left Motor Speeds:");
  Serial.println(motorLeftSpeed);
}

float Move(float rightSpeed, float leftSpeed, int rightDirection, int leftDirection) {
  // function that moves motor in the specified directions and speeds
  motorRight->setSpeed(rightSpeed);
  motorLeft->setSpeed(leftSpeed);
  motorRight->run(rightDirection); // backward default
  motorLeft->run(leftDirection); // forward default
}

// define pins for motors and sensors
#define MOTOR_RIGHT 1
#define MOTOR_LEFT 3
#define IR_RIGHT_IN A2
#define IR_LEFT_IN A1

// import the motor shield library to control motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <PID_v1.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorRight = AFMS.getMotor(MOTOR_RIGHT);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(MOTOR_LEFT);

// define variables that will control motor speed
float motorRightSpeed = 0;
float motorLeftSpeed = 0;
int IRSensorRight = 0;
int IRSensorLeft = 0;
int sensorDifference = 0;

// define variables that control PID
double Setpoint, Input, Output;
double Kp=1, Ki=1, Kd=1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(9600);
  pinMode(IR_RIGHT_IN, INPUT);
  pinMode(IR_LEFT_IN, INPUT);

  AFMS.begin();

  Setpoint = 0;
  myPID.SetMode(AUTOMATIC);
}

void loop() {
  // if (Serial.available() > 0) {
  //   // read the incoming byte:
  //   String word = Serial.readString();
  //   if (word == "start") {
  //     Move(motorRightSpeed, motorLeftSpeed);
  //   } else if (word == "stop") {
  //     stopMoving();
  //   }
  // }

  // -------------------------------------------------------------------------------

  IRSensorRight = analogRead(IR_RIGHT_IN);
  IRSensorLeft = -(analogRead(IR_LEFT_IN));
  // add other ir sensors later

  sensorDifference = IRSensorRight + IRSensorLeft;
  Input = sensorDifference;

  // sensorDifference = IRSensorLeft - IRSensorRight;
  // unsigned long time = millis();
  // double timeChange = (double)(now - )
  // Serial.println(Input);
  myPID.Compute();
  Serial.println("IRRIGHT");
  Serial.println(IRSensorRight);
  delay(200);
  Serial.println("IRLEFT");
  Serial.println(IRSensorLeft);
  delay(200);
  Serial.println("DIFFERENCE");
  Serial.println(Input);
  Serial.println("OUTPUT");
  Serial.println(Output);
  delay(200);
  // Serial.println(sensorDifference);
  // delay(200);

  // if (Output > 0) {
  //   motorRightSpeed = 50;
  //   motorLeftSpeed = 50;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // }

  motorRightSpeed = 50;
  motorLeftSpeed = 50;
  Move(motorRightSpeed, motorLeftSpeed);


  // if (sensorDifference > 20) {
  //   // turn right
  //   motorRightSpeed = motorRightSpeed * 1.5;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (sensorDifference < -20) {
  //   // turn left
  //   motorLeftSpeed = motorLeftSpeed * 1.5;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // }

// have to take in sign toooooo
  // if (Output >= 0 && Output < 50 && Input > 0) {
  //   // slight turn right
  //   motorRightSpeed = motorRightSpeed*1.5;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 0 && Output < 50 && Input < 0) {
  //   // slight turn left
  //   motorLeftSpeed = motorLeftSpeed*1.5;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 50 && Output < 100 && Input > 0) {
  //   // slightly bigger turn right
  //   motorRightSpeed = motorRightSpeed*2;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 50 && Output < 100 && Input < 0) {
  //   // slightly bigger turn left
  //   motorLeftSpeed = motorLeftSpeed*2;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 100 && Output < 200 && Input > 0) {
  //   // even bigger turn right
  //   motorRightSpeed = motorRightSpeed*2.5;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 100 && Output < 200 && Input < 0) {
  //   // even bigger turn left
  //   motorLeftSpeed = motorLeftSpeed*2.5;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 200 && Output < 256 && Input > 0) {
  //   // turnnnnnnnn right
  //   motorRightSpeed = motorRightSpeed*3;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else if (Output >= 200 && Output < 256 && Input < 0) {
  //   // turrrrrrrrrrn left
  //   motorLeftSpeed = motorLeftSpeed*3;
  //   Move(motorRightSpeed, motorLeftSpeed);
  // } else {
  //   Serial.println("What's going on.");
  // }
  // delay(100);
}

int Move(int rightSpeed, int leftSpeed) {
  motorRight->setSpeed(rightSpeed);
  motorLeft->setSpeed(leftSpeed);
  motorRight->run(BACKWARD);
  motorLeft->run(FORWARD);
}

void stopMoving() {
  motorRight->run(RELEASE);
  motorLeft->run(RELEASE);
}

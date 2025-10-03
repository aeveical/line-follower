// define pins for motors and sensors
#define MOTOR_RIGHT 4
#define MOTOR_LEFT 1
#define IR_RIGHT A5
#define IR_LEFT A4

// import the motor shield library to control motors
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *motorRight = AFMS.getMotor(MOTOR_RIGHT);
Adafruit_DCMotor *motorLeft = AFMS.getMotor(MOTOR_LEFT);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_LEFT, INPUT);

  AFMS.begin();
  // motorRight->setSpeed(100);
  // motorLeft->setSpeed(100);
  // motorRight->run(FORWARD);
  // motorLeft->run(FORWARD);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    // read the incoming byte:
    String word = Serial.readString();
    Serial.println(word);
  }
}

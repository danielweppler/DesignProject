//definitions
#include "definitions.h"

void setup() {

  Wire.begin();
  Serial.begin(9600);

  //ultrasonic setup
  pinMode(leftUltrasonicPing, OUTPUT);
  pinMode(leftUltrasonicData, INPUT);
  pinMode(middleUltrasonicPing, OUTPUT);
  pinMode(middleUltrasonicData, INPUT);
  pinMode(rightUltrasonicPing, OUTPUT);
  pinMode(rightUltrasonicData, INPUT);

  //motor setup
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);
  pinMode(liftMotorPin, OUTPUT);
  pinMode(winchMotorPin, OUTPUT);

  rightMotor.attach(rightMotorPin);
  leftMotor.attach(leftMotorPin);
  liftMotor.attach(liftMotorPin);
  winchMotor.attach(winchMotorPin);

  //encoder setup
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

}

void loop() {


}

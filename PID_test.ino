#include <Servo.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

#include <PID_v1.h>
double leftSetpoint, leftInput, leftOutput;
double Kp = 2, Ki = 5, Kd = 1;
double rightSetpoint, rightInput, rightOutput;

const int rightMotorPin = 8;
const int leftMotorPin = 9;



const int countsPerRev = 627.2;
const int revs = 3;

//objects
Servo rightMotor;
Servo leftMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);

//constant values like motor speeds/ positions
const int motorSpeed;
const int motorStop = 1500;
const int leftOffset = 0;
const int rightOffset = 0;

unsigned int rightMotorSpeed;
unsigned int leftMotorSpeed;


void setup() {
  // put your setup code here, to run once:

  Wire.begin();
  Serial.begin(9600);

  rightMotor.attach(rightMotorPin);
  leftMotor.attach(leftMotorPin);

  //motor setup
  pinMode(rightMotorPin, OUTPUT);
  pinMode(leftMotorPin, OUTPUT);


  //encoder setup
  encoder_LeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

}

void loop() {
  // put your main code here, to run repeatedly:

  leftInput = encoder_LeftMotor.getRawPosition();
  rightInput = encoder_RightMotor.getRawPosition();

  leftSetpoint = revs * countsPerRev;
  rightSetpoint = revs * countsPerRev;

  leftPID.SetOutputLimits(-500, 500);
  rightPID.SetOutputLimits(-500, 500);

  leftPID.Compute();
  rightPID.Compute();

  leftMotorSpeed = 1500 + leftOutput;
  rightMotorSpeed = 1500 + rightOutput;

  leftMotor.writeMicroseconds(leftMotorSpeed);
  rightMotor.writeMicroseconds(rightMotorSpeed);

}


#ifndef DEFINIIONS_H
#define DEFINIIONS_H

//debug lines
//#define DEBUG_ENCODERS

//libraries
#include <Servo.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

//PINS------------------------------------------------

const int leftUltrasonicPing = 2;   //input plug
const int leftUltrasonicData = 3;   //output plug
const int middleUltrasonicPing = 4;
const int middleUltrasonicData = 5;
const int rightUltrasonicPing = 6;
const int rightUltrasonicData = 7;
const int modeButton = 6;
const int rightMotorPin = 8;
const int leftMotorPin = 9;
const int liftMotorPin = 10;
const int winchMotorPin = 11;
const int ledPin = 13;
const int irPin = A3;
const int notUsed=12;



//constant values like motor speeds/ positions--------

const int motorSpeed = 1500;
const int motorStop = 1500;
const int leftOffset = 0;
const int rightOffset = 0;
const int countsPerRev = 627.2;
const int revs = 5;
const int blinkInterval = 1000;


//VARIABLES------------------

unsigned long leftEchoTime;
unsigned long middleEchoTime;
unsigned long rightEchoTime;
unsigned int rightMotorSpeed;
unsigned int leftMotorSpeed;

//pid
double leftSetpoint, leftInput, leftOutput;
double Kp = 1.01, Ki = 0.02, Kd = 0.115;
double turnKp = 1.4, turnKi = 0.2, turnKd = 0.2;
double rightSetpoint, rightInput, rightOutput;

//led
unsigned long ledMillis = 0;
bool ledState = true;

//IR
char irInput;

//mode functionality
bool buttonState;
bool buttonPreviousState = false;
int mode = 0;

//State functionality
int currentState=0;

//start delay
bool startDelayed= false;
int twoSecTimer;

//case based initialization stuff
bool didOnce = false;
unsigned long leftStartCount=0;
unsigned long rightStartCount=0;

//objects
Servo rightMotor;
Servo leftMotor;
Servo liftMotor;
Servo winchMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

SoftwareSerial IRSerial(irPin, notUsed); // RX, TX

PID leftPID(&leftInput, &leftOutput, &leftSetpoint, Kp, Ki, Kd, DIRECT);
PID rightPID(&rightInput, &rightOutput, &rightSetpoint, Kp, Ki, Kd, DIRECT);



//HEADERS-------------
#include "ping.h"
#include "beaconSensing.h"
#include "driveFunctions.h"

#endif

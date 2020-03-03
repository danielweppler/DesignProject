
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

//pins
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


//objects
Servo rightMotor;
Servo leftMotor;
Servo liftMotor;
Servo winchMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;
SoftwareSerial IRSerial(irPin, notUsed); // RX, TX


//constant values like motor speeds/ positions
const int motorSpeed = 1500;
const int motorStop = 1500;
const int leftOffset = 250;
const int rightOffset = 0;

const int blinkInterval = 1000;
//

//variables
unsigned long leftEchoTime;
unsigned long middleEchoTime;
unsigned long rightEchoTime;

unsigned int rightMotorSpeed;
unsigned int leftMotorSpeed;

unsigned long ledMillis = 0;
bool ledState = true;

char irInput;

//mode functionality
bool buttonState;
bool buttonPreviousState = false;
int mode = 0;


//headers
#include "ping.h"
#include "beaconSensing.h"

#endif
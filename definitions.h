
#ifndef DEFINIIONS_H
#define DEFINIIONS_H

//libraries
#include <Servo.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>
#include <uSTimer2.h>


//objects
Servo rightMotor;
Servo leftMotor;
Servo liftMotor;
Servo winchMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

//pins
const int leftUltrasonicPing = 2;   //input plug
const int leftUltrasonicData = 3;   //output plug
const int middleUltrasonicPing = 4;
const int middleUltrasonicData = 5;
const int rightUltrasonicPing = 6;
const int rightUltrasonicData = 7;
const int modeButton = 7;
const int rightMotorPin = 8;
const int leftMotorPin = 9;
const int liftMotorPin = 10;
const int winchMotorPin = 11;
const int ledPin = 13;

//constant values like motor speeds/ positions
const int motorSpeed = 1600;
const int motorStop = 1500;
const int leftOffset;
const int rightOffset;

const int blinkInterval=1000;
//

//variables
unsigned long leftEchoTime;
unsigned long middleEchoTime;
unsigned long rightEchoTime;

unsigned long ledMillis =0;
bool ledState = true;

//mode functionality
bool buttonState;
bool buttonPreviousState = false;
int mode=0;


//headers
#include "ping.h"

#endif

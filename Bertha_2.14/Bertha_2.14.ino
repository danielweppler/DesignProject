//definitions
#include "definitions.h"

void setup() {

  Wire.begin();
  Serial.begin(9600);
  IRSerial.begin(2400);

  //mode button setup
  pinMode(modeButton, INPUT_PULLUP);

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

  leftPID.SetMode(AUTOMATIC);
  rightPID.SetMode(AUTOMATIC);

  leftPID.SetSampleTime(100);

}

void loop() {

  //2 second delay for modes
  if ((millis() - twoSecTimer) > 2000) {
    startDelayed = true;
  }

  // mode functionality
  buttonState = !digitalRead(modeButton);

  if (buttonState && buttonState != buttonPreviousState)
  {
    buttonPreviousState = buttonState;
    mode++;
    mode = mode & 3;
    twoSecTimer = millis();
    startDelayed = false;
  }
  else if (!buttonState && buttonState != buttonPreviousState)
  {
    buttonPreviousState = buttonState;
  }

  switch (mode) {

    case 0:
      {
        irCheck(); //gives irInput the value of the character being measured should be 0 and 5

        leftMotor.writeMicroseconds(motorStop);
        rightMotor.writeMicroseconds(motorStop);

        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();

        //        leftPing();
        //        Serial.print("LeftEcho");
        //        Serial.println(leftEchoTime);

        break;
      }

    //driving functions
    case 1:
      {
        if (startDelayed) {
          leftInput = encoder_LeftMotor.getRawPosition();
          rightInput = encoder_RightMotor.getRawPosition();

          leftSetpoint = revs * countsPerRev;
          rightSetpoint = revs * countsPerRev;

          leftPID.SetOutputLimits(-450, 450);
          rightPID.SetOutputLimits(-450, 450);

          leftPID.Compute();
          rightPID.Compute();


#ifdef DEBUG_ENCODERS

          Serial.print("Encoders L: ");
          Serial.print(encoder_LeftMotor.getRawPosition());
          Serial.print(", R: ");
          Serial.println(encoder_RightMotor.getRawPosition());
#endif
        }

        break;
      }

    case 2:
      {
        if (startDelayed) {

          if (!didOnce) {
            didOnce = true;
            leftStartCount = encoder_LeftMotor.getRawPosition();
            rightStartCount = encoder_RightMotor.getRawPosition();

            leftPID.SetTunings(turnKp, turnKi, turnKd);
            rightPID.SetTunings(turnKp, turnKi, turnKd);
          }

          leftInput = encoder_LeftMotor.getRawPosition();
          rightInput = encoder_RightMotor.getRawPosition();

          switch (currentState) {

            case 0: {

                irCheck(); //start checking for ir signal

                leftPID.SetOutputLimits(-100, 100);
                rightPID.SetOutputLimits(-100, 100);

                if (irInput == 5) {        //if its the correct signal update state
                  
                  didOnce = false;
                  currentState++;
                  
                }
                
                else {
                                      
                    rightSetpoint = rightStartCount + turnCalc(90);
                    leftSetpoint = leftStartCount - turnCalc(90);     //first turn 90 degrees left

                    leftPID.Compute();
                    rightPID.Compute();

                    if (leftInput < (leftStartCount - turnCalc(90))) {

                      currentState++;
                    
                    }
                  
                }
                break;
              }

            case 1: { //here because we saw the signal OR turned left 90


                if (!didOnce) {
                  didOnce = true;

                  leftStartCount = encoder_LeftMotor.getRawPosition();    //get new positions (after turning to find beacon)
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  leftPID.SetTunings(turnKp, turnKi, turnKd);
                  rightPID.SetTunings(turnKp, turnKi, turnKd);
                }

                irCheck(); //keep looking for signal

                leftPID.SetOutputLimits(-100, 100);
                rightPID.SetOutputLimits(-100, 100);

                 if (irInput == 5) {        //if its the correct signal update state
                  
                  didOnce = false;
                  currentState++;
                  
                }

                else {

                  rightSetpoint = rightStartCount - turnCalc(90);
                  leftSetpoint = leftStartCount + turnCalc(90);     //now turn 90 degrees right

                  leftPID.Compute();
                  rightPID.Compute();

                  if (rightInput < (rightStartCount - turnCalc(90))) {

                    currentState++;
                    
                  }
                }

                break;

            }

            case 2: {

              
                            
            }

        }

        leftMotorSpeed = 1500 + leftOutput;
        rightMotorSpeed = 1500 + rightOutput;

        leftMotor.writeMicroseconds(leftMotorSpeed);
        rightMotor.writeMicroseconds(rightMotorSpeed);

        }


      break;
  }

default: {

    leftMotor.writeMicroseconds(motorStop);
    rightMotor.writeMicroseconds(motorStop);
  }


  if ((millis() - ledMillis) > blinkInterval) {
    ledMillis = millis();
    digitalWrite(ledPin, ledState);
    ledState != ledState;
  }

}


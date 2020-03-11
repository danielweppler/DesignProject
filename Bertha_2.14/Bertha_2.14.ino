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

  //how often PID does calculations (not determined by the compute functions)
  leftPID.SetSampleTime(50);

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
    mode = mode & 7;
    twoSecTimer = millis();
    startDelayed = false;
    didOnce = false;
    onlyDidOnce = false;
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

        if ((millis() - ledMillis) > blinkInterval) {

#ifdef DEBUG_IRANDULTRASONIC
          Serial.print("input read:    ");
          Serial.println(irInput);
          Serial.print("middleEcho: ");
          Serial.println(middleEchoTime / 58);
          middlePing();
#endif
        }

        break;
      }

    //driving functions

    case 1:// just drive straight bro
      {
        if (startDelayed) {

          //uncomment to test turning

          //          leftPID.SetTunings(turnKp, turnKi, turnKd);
          //          rightPID.SetTunings(turnKp, turnKi, turnKd);

          leftInput = encoder_LeftMotor.getRawPosition();
          rightInput = encoder_RightMotor.getRawPosition();

          //go forwards this many revolutions
          leftSetpoint = revs * countsPerRev;
          rightSetpoint = revs * countsPerRev;

          leftPID.SetOutputLimits(-450, 450);
          rightPID.SetOutputLimits(-450, 450);

          //PID library does math
          leftPID.Compute();
          rightPID.Compute();

          //left and right motor speed from PID
          leftMotorSpeed = 1500 + leftOutput;
          rightMotorSpeed = 1500 + rightOutput;

          // send the speeds to the motors
          leftMotor.writeMicroseconds(leftMotorSpeed);
          rightMotor.writeMicroseconds(rightMotorSpeed);

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

          }

          leftMotorSpeed = 1500 + leftOutput;
          rightMotorSpeed = 1500 + rightOutput;

          leftMotor.writeMicroseconds(leftMotorSpeed);
          rightMotor.writeMicroseconds(rightMotorSpeed);

        }


        break;
      }

    case 3://just spin till you see the sensor fam
      {
        if (startDelayed) {

          //zeroes motors and stores total distance in front of bot on
          if (!onlyDidOnce) {
            onlyDidOnce = true;
            currentState = 0;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
          }

          switch (currentState) {

             case 0://spin around a few times until IR sensor is read
              {

                if (!didOnce) {
                  didOnce = true;

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //set target of PID as 1080 degrees to the right
                  leftSetpoint = leftStartCount + turnCalc(1080);
                  rightSetpoint = rightStartCount - turnCalc(1080);

                  //setting speeds (make this smaller to spin slower, might have to tune turnPID values but maybe not, if needed try reducing turnKd by increments of 0.02)
                  leftPID.SetOutputLimits(-100, 100);
                  rightPID.SetOutputLimits(-100, 100);

                  //setting the tuning for PID
                  leftPID.SetTunings(turnKp, turnKi, turnKd);
                  rightPID.SetTunings(turnKp, turnKi, turnKd);
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                //update IR values
                irCheck();

                //increments states when IR is found
                if (irInput == '5' || irInput == '0' || irInput == 53 ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                  beaconDelay = millis();

                  leftTurned = leftInput - leftStartCount;
                  rightTurned = rightInput - rightStartCount;

                  if (rightTurned > turnCalc(360) && leftTurned > turnCalc(360)) {
                    rightTurned -= turnCalc(360);
                    leftTurned -= turnCalc(360);
                  }
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 1:// go straight
              {

                //updates front distance
                middlePing();

                //just chills for 1.5 seconds hopeing US values arent trash
                if (!didOnce && (millis() - beaconDelay) > 1500) {

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //find forward distance
                  thirdFrontDist = middleEchoTime / 58;

                  //go forward measured ultrasonic distance plus 5 more to be sure
                  leftSetpoint = leftStartCount + distanceCalc(thirdFrontDist + 10);
                  rightSetpoint = rightStartCount + distanceCalc(thirdFrontDist + 10);

                  //setting speeds
                  leftPID.SetOutputLimits(-450, 450);
                  rightPID.SetOutputLimits(-450, 450);

                  //setting the tuning for PID
                  leftPID.SetTunings(Kp, Ki, Kd);
                  rightPID.SetTunings(Kp, Ki, Kd);

                  didOnce = true;
                }

                //only do this after top part is excecuted
                else {

                  //reads updated encoder values
                  leftInput = encoder_LeftMotor.getRawPosition();
                  rightInput = encoder_RightMotor.getRawPosition();

                  //PID library does math
                  leftPID.Compute();
                  rightPID.Compute();

                  //left and right motor speed from PID
                  leftMotorSpeed = 1500 + leftOutput;
                  rightMotorSpeed = 1500 + rightOutput;


                  //if the bot isnt pointing in the right direction go back to last case and find it again
                  //as long as bot isnt too close (30 cm)

                  //                if (irInput != 53 && (middleEchoTime / 58) > 30) {
                  //                  currentState = 3;
                  //                  didOnce = false;
                  //                  leftMotorSpeed = 1500;
                  //                  rightMotorSpeed = 1500;
                  //                }


                  //if beacon is off and front distance is +- 5 cm from the targeted distance (should still be more than enough in theory)
                  // else if ( irInput != 53 && (leftInput) >= (leftSetpoint - distanceCalc(5)) && (leftInput) <= (leftSetpoint + distanceCalc(5)) ) {

                  else if (irInput != 53 && middleEchoTime / 58 < 6 ) {
                    currentState++;
                    didOnce = false;
                    leftMotorSpeed = 1500;
                    rightMotorSpeed = 1500;
                    lastLeftStartCount = leftStartCount;
                    lastRightStartCount = rightStartCount;
                  }

                }
                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 2://reverse to about where you were done turning
              {
                if (!didOnce) {

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //go reverse to starting point of last state
                  leftSetpoint = lastLeftStartCount;
                  rightSetpoint = lastRightStartCount;

                  //setting speeds
                  leftPID.SetOutputLimits(-450, 450);
                  rightPID.SetOutputLimits(-450, 450);

                  //setting the tuning for PID
                  leftPID.SetTunings(Kp, Ki, Kd);
                  rightPID.SetTunings(Kp, Ki, Kd);

                  didOnce = true;
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                if ( (leftInput) >= (leftSetpoint - distanceCalc(2)) && (leftInput) <= (leftSetpoint + distanceCalc(2)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 3:
              {

                if (!didOnce) {
                  didOnce = true;

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //turn around about perpendicular to the bridge
                  leftSetpoint = leftStartCount-leftTurned+turnCalc(180);
                  rightSetpoint = rightStartCount+rightTurned-turnCalc(180);

                  //setting speeds
                  leftPID.SetOutputLimits(-150, 150);
                  rightPID.SetOutputLimits(-150, 150);

                  //setting the tuning for PID
                  leftPID.SetTunings(turnKp, turnKi, turnKd);
                  rightPID.SetTunings(turnKp, turnKi, turnKd);
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                //if rotated +- 1 target degrees, increment state 
                if ( (leftInput) >= (leftSetpoint - turnCalc(1)) && (leftInput) <= (leftSetpoint + turnCalc(1)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }


            default:
              {
                leftMotor.writeMicroseconds(motorStop);
                rightMotor.writeMicroseconds(motorStop);
              }

              ///End of state switch
          }



          ////end of mode 3 before break and bracket
        }
        break;
      }

    case 4://main code
      {
        if (startDelayed) {

          //zeroes motors and stores total distance in front of bot on
          if (!onlyDidOnce) {
            onlyDidOnce = true;
            currentState = 0;
            encoder_LeftMotor.zero();
            encoder_RightMotor.zero();
            middlePing();
            initialFrontDist = middleEchoTime / 58;
          }

          switch (currentState) {

            case 0: {

                if (!didOnce) {
                  didOnce = true;
                  //set target of PID as half of the box
                  leftSetpoint = distanceCalc(110);
                  rightSetpoint = distanceCalc(110);

                  //setting speeds
                  leftPID.SetOutputLimits(-450, 450);
                  rightPID.SetOutputLimits(-450, 450);

                  //setting the tuning for PID
                  leftPID.SetTunings(Kp, Ki, Kd);
                  rightPID.SetTunings(Kp, Ki, Kd);
                }

                //updates middleUltrasonic
                middlePing();

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;



                //if front distance is +- 5 cm from the half way mark of the track
                //                if ( (middleEchoTime / 58) >= ((initialFrontDist / 2) - 5) && (middleEchoTime / 58) <= ((initialFrontDist / 2) + 5) ) {
                //                  currentState++;
                //                  didOnce = false;
                //                  leftMotorSpeed = 1500;
                //                  rightMotorSpeed = 1500;
                //                }

                if ( (leftInput) >= (leftSetpoint - distanceCalc(2)) && (leftInput) <= (leftSetpoint + distanceCalc(2)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);


                break;
              }
            case 1://turn right 90 degrees
              {

                if (!didOnce) {
                  didOnce = true;

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //set target of PID as 90 degrees to the right
                  leftSetpoint = leftStartCount + turnCalc(90);
                  rightSetpoint = rightStartCount - turnCalc(90);

                  //setting speeds
                  leftPID.SetOutputLimits(-100, 100);
                  rightPID.SetOutputLimits(-100, 100);

                  //setting the tuning for PID
                  leftPID.SetTunings(turnKp, turnKi, turnKd);
                  rightPID.SetTunings(turnKp, turnKi, turnKd);
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                //if rotated +- 2 degrees of 90 deg, increment state (could use left or right motor values, doesnt matter)
                if ( (leftInput) >= (leftSetpoint - turnCalc(2)) && (leftInput) <= (leftSetpoint + turnCalc(2)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 2:// go striaght till half way again
              {

                if (!didOnce) {
                  didOnce = true;

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //find forward distance
                  middlePing();
                  secondFrontDist = middleEchoTime / 58;

                  //set target of PID as halfway of forward direction
                  leftSetpoint = leftStartCount + distanceCalc(120);
                  rightSetpoint = leftStartCount + distanceCalc(120);

                  //setting speeds
                  leftPID.SetOutputLimits(-450, 450);
                  rightPID.SetOutputLimits(-450, 450);

                  //setting the tuning for PID
                  leftPID.SetTunings(Kp, Ki, Kd);
                  rightPID.SetTunings(Kp, Ki, Kd);
                }

                //updates front distance
                //middlePing();

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                //if front distance is +- 5 cm from the half way mark of the forward distance
                //                if ( (middleEchoTime / 58) >= ((secondFrontDist / 2) - 5) && (middleEchoTime / 58) <= ((secondFrontDist / 2) + 5) ) {
                //                  currentState++;
                //                  didOnce = false;
                //                  leftMotorSpeed = 1500;
                //                  rightMotorSpeed = 1500;
                //                }

                if ( (leftInput) >= (leftSetpoint - distanceCalc(2)) && (leftInput) <= (leftSetpoint + distanceCalc(2)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);


                break;
              }

            case 3://spin around a few times until IR sensor is read
              {

                if (!didOnce) {
                  didOnce = true;

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //set target of PID as 1080 degrees to the right
                  leftSetpoint = leftStartCount + turnCalc(1080);
                  rightSetpoint = rightStartCount - turnCalc(1080);

                  //setting speeds (make this smaller to spin slower, might have to tune turnPID values but maybe not, if needed try reducing turnKd by increments of 0.02)
                  leftPID.SetOutputLimits(-100, 100);
                  rightPID.SetOutputLimits(-100, 100);

                  //setting the tuning for PID
                  leftPID.SetTunings(turnKp, turnKi, turnKd);
                  rightPID.SetTunings(turnKp, turnKi, turnKd);
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                //update IR values
                irCheck();

                //increments states when IR is found
                if (irInput == '5' || irInput == '0' || irInput == 53 ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                  beaconDelay = millis();

                  leftTurned = leftInput - leftStartCount;
                  rightTurned = rightInput - rightStartCount;

                  if (rightTurned > turnCalc(360) && leftTurned > turnCalc(360)) {
                    rightTurned -= turnCalc(360);
                    leftTurned -= turnCalc(360);
                  }
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 4:// go straight
              {

                //updates front distance
                middlePing();

                //just chills for 1.5 seconds hopeing US values arent trash
                if (!didOnce && (millis() - beaconDelay) > 1500) {

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //find forward distance
                  thirdFrontDist = middleEchoTime / 58;

                  //go forward measured ultrasonic distance plus 5 more to be sure
                  leftSetpoint = leftStartCount + distanceCalc(thirdFrontDist + 10);
                  rightSetpoint = rightStartCount + distanceCalc(thirdFrontDist + 10);

                  //setting speeds
                  leftPID.SetOutputLimits(-450, 450);
                  rightPID.SetOutputLimits(-450, 450);

                  //setting the tuning for PID
                  leftPID.SetTunings(Kp, Ki, Kd);
                  rightPID.SetTunings(Kp, Ki, Kd);

                  didOnce = true;
                }

                //only do this after top part is excecuted
                else {

                  //reads updated encoder values
                  leftInput = encoder_LeftMotor.getRawPosition();
                  rightInput = encoder_RightMotor.getRawPosition();

                  //PID library does math
                  leftPID.Compute();
                  rightPID.Compute();

                  //left and right motor speed from PID
                  leftMotorSpeed = 1500 + leftOutput;
                  rightMotorSpeed = 1500 + rightOutput;


                  //if the bot isnt pointing in the right direction go back to last case and find it again
                  //as long as bot isnt too close (30 cm)

                  //                if (irInput != 53 && (middleEchoTime / 58) > 30) {
                  //                  currentState = 3;
                  //                  didOnce = false;
                  //                  leftMotorSpeed = 1500;
                  //                  rightMotorSpeed = 1500;
                  //                }


                  //if beacon is off and front distance is +- 5 cm from the targeted distance (should still be more than enough in theory)
                  // else if ( irInput != 53 && (leftInput) >= (leftSetpoint - distanceCalc(5)) && (leftInput) <= (leftSetpoint + distanceCalc(5)) ) {

                  else if (irInput != 53 && middleEchoTime / 58 < 6 ) {
                    currentState++;
                    didOnce = false;
                    leftMotorSpeed = 1500;
                    rightMotorSpeed = 1500;
                    lastLeftStartCount = leftStartCount;
                    lastRightStartCount = rightStartCount;
                  }

                }
                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 5://reverse to about where you were done turning
              {
                if (!didOnce) {

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //go reverse to starting point of last state
                  leftSetpoint = lastLeftStartCount;
                  rightSetpoint = lastRightStartCount;

                  //setting speeds
                  leftPID.SetOutputLimits(-450, 450);
                  rightPID.SetOutputLimits(-450, 450);

                  //setting the tuning for PID
                  leftPID.SetTunings(Kp, Ki, Kd);
                  rightPID.SetTunings(Kp, Ki, Kd);

                  didOnce = true;
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                if ( (leftInput) >= (leftSetpoint - distanceCalc(2)) && (leftInput) <= (leftSetpoint + distanceCalc(2)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }

            case 6:
              {

                if (!didOnce) {
                  didOnce = true;

                  //stores starting encoder position for this state
                  leftStartCount = encoder_LeftMotor.getRawPosition();
                  rightStartCount = encoder_RightMotor.getRawPosition();

                  //turn around about perpendicular to the bridge
                  leftSetpoint = leftStartCount-leftTurned+turnCalc(180);
                  rightSetpoint = rightStartCount+rightTurned-turnCalc(180);

                  //setting speeds
                  leftPID.SetOutputLimits(-150, 150);
                  rightPID.SetOutputLimits(-150, 150);

                  //setting the tuning for PID
                  leftPID.SetTunings(turnKp, turnKi, turnKd);
                  rightPID.SetTunings(turnKp, turnKi, turnKd);
                }

                //reads updated encoder values
                leftInput = encoder_LeftMotor.getRawPosition();
                rightInput = encoder_RightMotor.getRawPosition();

                //PID library does math
                leftPID.Compute();
                rightPID.Compute();

                //left and right motor speed from PID
                leftMotorSpeed = 1500 + leftOutput;
                rightMotorSpeed = 1500 + rightOutput;

                //if rotated +- 1 target degrees, increment state 
                if ( (leftInput) >= (leftSetpoint - turnCalc(1)) && (leftInput) <= (leftSetpoint + turnCalc(1)) ) {
                  currentState++;
                  didOnce = false;
                  leftMotorSpeed = 1500;
                  rightMotorSpeed = 1500;
                }

                // send the speeds to the motors
                leftMotor.writeMicroseconds(leftMotorSpeed);
                rightMotor.writeMicroseconds(rightMotorSpeed);

                break;
              }


            default://for state switch
              {
                leftMotor.writeMicroseconds(motorStop);
                rightMotor.writeMicroseconds(motorStop);
              }

              ///End of state switch
          }



          ////end of mode 4 before break and bracket
        }
        break;
      }

    default: {
        leftMotor.writeMicroseconds(motorStop);
        rightMotor.writeMicroseconds(motorStop);
      }

  }

  if ((millis() - ledMillis) > blinkInterval) {
    ledMillis = millis();
    digitalWrite(ledPin, ledState);
    ledState != ledState;
  }

}

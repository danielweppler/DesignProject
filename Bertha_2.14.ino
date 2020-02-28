//definitions
#include "definitions.h"

void setup() {

  Wire.begin();
  Serial.begin(9600);

  //mode button setup
  pinMode(modeButton, INPUT_PULLUP);

  //ultrasonic setup
  //  pinMode(leftUltrasonicPing, OUTPUT);
  //  pinMode(leftUltrasonicData, INPUT);
  //  pinMode(middleUltrasonicPing, OUTPUT);
  //  pinMode(middleUltrasonicData, INPUT);
  //  pinMode(rightUltrasonicPing, OUTPUT);
  //  pinMode(rightUltrasonicData, INPUT);

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

  // mode functionality
  buttonState = !digitalRead(modeButton);

  if (buttonState && buttonState != buttonPreviousState)
  {
    buttonPreviousState = buttonState;
    mode++;
    mode = mode & 3;
  }
  else if (!buttonState && buttonState != buttonPreviousState)
  {
    buttonPreviousState = buttonState;
  }

  switch (mode) {

    case 0:
      {
        leftMotor.writeMicroseconds(motorStop);
        rightMotor.writeMicroseconds(motorStop);

        encoder_LeftMotor.zero();
        encoder_RightMotor.zero();
        break;
      }

    case 1:
      {
        leftMotorSpeed  = motorSpeed + leftOffset;
        rightMotorSpeed = motorSpeed + rightOffset;

        leftMotor.writeMicroseconds(leftMotorSpeed);
        rightMotor.writeMicroseconds(rightMotorSpeed);

#ifdef DEBUG_ENCODERS

        Serial.print("Encoders L: ");
        Serial.print(encoder_LeftMotor.getRawPosition());
        Serial.print(", R: ");
        Serial.println(encoder_RightMotor.getRawPosition());
#endif

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

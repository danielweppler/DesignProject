#ifndef PING_H
#define PING_H


void leftPing() {

  digitalWrite(leftUltrasonicPing, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(leftUltrasonicPing, LOW);

  leftEchoTime = pulseIn(leftUltrasonicData, HIGH, 10000);

}

void middlePing() {

  digitalWrite(middleUltrasonicPing, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(middleUltrasonicPing, LOW);

  middleEchoTime = pulseIn(middleUltrasonicData, HIGH, 10000);

}

void rightPing() {

  digitalWrite(rightUltrasonicPing, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(rightUltrasonicPing, LOW);

  rightEchoTime = pulseIn(rightUltrasonicData, HIGH, 10000);

}


#endif

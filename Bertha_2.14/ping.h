#ifndef PING_H
#define PING_H


void leftPing() {

  digitalWrite(leftUltrasonicPing, LOW);
  delayMicroseconds(5);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(leftUltrasonicPing, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(leftUltrasonicPing, LOW);

  leftEchoTime = pulseIn(leftUltrasonicData, HIGH, 25000);

}

void middlePing() {
  digitalWrite(leftUltrasonicPing, LOW);
  delayMicroseconds(5);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(middleUltrasonicPing, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(middleUltrasonicPing, LOW);


  middleTempTime = pulseIn(middleUltrasonicData, HIGH, 25000);

  if(middleTempTime >20){
    middleEchoTime = middleTempTime;
    }


}

void rightPing() {
  digitalWrite(leftUltrasonicPing, LOW);
  delayMicroseconds(5);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(rightUltrasonicPing, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(rightUltrasonicPing, LOW);

  rightEchoTime = pulseIn(rightUltrasonicData, HIGH, 25000);

}


#endif

#ifndef DRIVEFUNCTIONS_H
#define DRIVEFUNCTIONS_H

double turnCalc (int angle) {
  
double encoderCounts;
double wheelBase=0.214;
double wheelDiameter=0.05;

encoderCounts = (countsPerRev*wheelBase*angle*3.14159265/180)/(2*3.14159265*wheelDiameter);

return encoderCounts;
}

double distanceCalc(double distance)
{
  //given distance in centimeters returns the necessary encoder counts
  
  double wheelDiameter = 5;  
  double encoderCounts =0;
  double pi = 3.14159265;

  encoderCounts = (countsPerRev * distance)/(pi*wheelDiameter);

  return encoderCounts;
  }
  


#endif

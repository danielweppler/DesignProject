#ifndef DRIVEFUNCTIONS_H
#define DRIVEFUNCTIONS_H

double turnCalc (int angle) {
  
double encoderCounts;
double wheelBase=0.214;
double wheelDiameter=0.102;

encoderCounts = (629.2*wheelBase*angle*3.14159265/180)/(2*3.14159265*wheelDiameter);

return encoderCounts;
}

double straightCalc(int distance)
{
  
  double encoderCounts =0;

  return encoderCounts;
  }


#endif

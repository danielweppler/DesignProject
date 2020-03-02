#ifndef BEACONSENSING_H
#define BEACONSENSING_H

void irCheck (char * irChar){
  if (IRSerial.available())
  {
    *irChar = IRSerial.read();
  }
  } 


#endif

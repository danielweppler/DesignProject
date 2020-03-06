#ifndef BEACONSENSING_H
#define BEACONSENSING_H

void irCheck (){
  if (IRSerial.available())
  {
    irInput = IRSerial.read();
  }
  } 


#endif

#ifndef BEACONSENSING_H
#define BEACONSENSING_H

void irCheck () {
  if (IRSerial.available())
  {
    irInput = IRSerial.read();
    
#ifdef DEBUG_IRANDULTRASONIC
    Serial.print("input read:    ");
    Serial.println(irInput);
#endif
  }
  else {
    irInput =69;
}
}


#endif

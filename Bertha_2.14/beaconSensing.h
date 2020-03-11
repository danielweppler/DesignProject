#ifndef BEACONSENSING_H
#define BEACONSENSING_H

void irCheck () {
  if (IRSerial.available())
  {
    irInput = IRSerial.read();
    
#ifdef DEBUG_IRANDULTRASONIC
    Serial.print("input read:    ");
    Serial.println(irInput);
    Serial.print("Middle dist:");
    Serial.println(middleEchoTime/58);
#endif
  }
  else {
    irInput =69;
}
}


#endif

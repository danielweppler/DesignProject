#ifndef BEACONSENSING_H
#define BEACONSENSING_H

void irCheck () {
  if (IRSerial.available())
  {
    irInput = IRSerial.read();
    Serial.print("input read:    ");
    Serial.println(irInput);
  }
  else {
    irInput =69;
}
}


#endif

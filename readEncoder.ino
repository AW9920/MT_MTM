void readEncoder(unsigned int *OutData, unsigned int DO, int CSn, unsigned int CLK, int i) {
  *OutData = 0;  //Reset Output Array.
  //Serial.print("Read PIN:\t"); Serial.println(DO1+i);
  //Serial.println(i);
  digitalWrite(CSn, LOW);
  delayMicroseconds(1);  //Waiting for Tclkfe=500ns Or 16 times NOP
  //Passing 12 times, from 0 to 11
  for (int x = 0; x < 12; x++) {
    digitalWrite(CLK, LOW);
    delayMicroseconds(1);  //Tclk/2_min = 500ns = 8 * 62.5ns
    digitalWrite(CLK, HIGH);
    delayMicroseconds(1);  //Tdo valid, like Tclk/2
    *OutData = (*OutData << 1) | digitalRead(DO);
  }
  digitalWrite(CSn, HIGH);  //deselects the encoder from reading
  //Serial.print("Output Data:\t"); Serial.println(*OutData);
}
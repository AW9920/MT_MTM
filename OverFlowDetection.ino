void OverFlowDetection(unsigned int *temp, unsigned int *pre, bool *rollover, bool *rollunder, int *counter, int i) {
  //Function variables
  int dif;

  //Over- or Underflow detection
  dif = (int)*temp - (int)*pre;
  //Serial.println(dif);

  if (dif < low_lim) {
    rollover[i] = true;
    rollunder[i] = false;
    counter[i]++;
    //Serial.println("Rollover");
  } else if (dif > high_lim) {
    rollover[i] = false;
    rollunder[i] = true;
    counter[i]--;
    //Serial.println("Rollunder");
  } else {
    rollover[i] = false;
    rollunder[i] = false;
  }

  //Incrementation on over- or underflow
  //inc_sensor_value = (int)*temp + factor * gain;
}
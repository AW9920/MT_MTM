void OverFlowDetection(unsigned int *temp, unsigned int *pre, bool *rollover, bool *rollunder, int *counter) {
  //Function variables
  int dif;

  //Over- or Underflow detection
  dif = (int)*temp - (int)*pre;
  //Serial.println(dif);

  if (dif < low_lim) {
    *rollover = true;
    *rollunder = false;
    *counter = *counter + 1;
    //Serial.println("Rollover");
  } else if (dif > high_lim) {
    *rollover = false;
    *rollunder = true;
    *counter = *counter - 1;
    //Serial.println("Rollunder");
  } else {
    *rollover = false;
    *rollunder = false;
  }

  //Incrementation on over- or underflow
  //inc_sensor_value = (int)*temp + factor * gain;
}
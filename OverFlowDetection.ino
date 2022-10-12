int OverFlowDetection(unsigned int *temp, unsigned int *pre) {
  //Function variables
  int dif;
  int factor;
  int inc_sensor_value;

  //Over- or Underflow detection
  dif = (int)*temp - (int)*pre;
  //Serial.println(dif);

  if (dif < low_lim) {
    factor = 1;
  } else if (dif > high_lim) {
    factor = -1;
  } else {
    factor = 0;
  }

  //Incrementation on over- or underflow
  inc_sensor_value = (int)*temp + factor * gain;

  return inc_sensor_value;
}
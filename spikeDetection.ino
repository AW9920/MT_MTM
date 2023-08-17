Quaternion spikeDetection(Quaternion* qxn, Quaternion* qyn1, float* d, int j) {
  //=======================================================
  //======            FUNCTION Variables            =======
  //=======================================================
  int n = 2;      //counter until a spike is accepted as correct; Define zero to comfortably turn filter off
  float T = 0.4;  //Threshold for which the raw value is replaced with the last safe value

  //Empty container for filtered values
  Quaternion q;

  //Write values of quaternion class into float[4] array
  //current versus recent save value
  float dif[4] = {
    qxn->w - qyn1->w,
    qxn->x - qyn1->x,
    qxn->y - qyn1->y,
    qxn->z - qyn1->z
  };
  //Empty conatiner for filter output
  float yn[4] = {
    q.w,
    q.x,
    q.y,
    q.z
  };
  //recent save value
  float yn1[4] = {
    qyn1->w,
    qyn1->x,
    qyn1->y,
    qyn1->z
  };
  // Current sensor data
  float xn[4] = {
    qxn->w,
    qxn->x,
    qxn->y,
    qxn->z
  };

  // Look for unreasonable peaks that may indicate flaut received sensor data
  for (int i = 0; i < 4; i++) {
    if ((abs(dif[i]) > T) && (c[j][i] < n)) {
      // recent big change:  hold previous safe value
      yn[i] = yn1[i];
      c[j][i] = c[j][i] + 1;
    } else {
      // normal operation, or else the recent big change must be real after all
      yn1[i] = xn[i];
      yn[i] = xn[i];
      c[j][i] = 0;
    }
  }

  //Update varibales
  q.w = yn[0];
  q.x = yn[1];
  q.y = yn[2];
  q.z = yn[3];

  UpdateQwF(qyn1, yn1);

  n = sizeof(dif) / sizeof(dif[0]);
  updateArray(d, dif, n);

  return q;
}
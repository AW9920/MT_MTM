void spikeDetection(Quaternion* qxn, Quaternion* qyn1) {
  //=======================================================
  //======            FUNCTION Variables            =======
  //=======================================================
  int c = 2;    //counter until a spike is accepted as correct; Define zero to comfortably turn filter off
  float T = 0;  //Threshold for which the raw value is replaced with the last safe value

  // Look for unreasonable peaks that may indicate flaut received sensor data
  if (abs(qxn->w - qyn1->w) >= T && (n < c)) {
    qxn->w = qyn1->w;
    n = n + 1;
  } else {
    qyn1->w = qxn->w;
    n = 0;
  }

  if (abs(qxn->x - qyn1->x) >= T && (n < c)) {
    qxn->x = qyn1->x;
    n = n + 1;
  } else {
    qyn1->x = qxn->x;
    n = 0;
  }

  if (abs(qxn->y - qyn1->y) >= T && (n < c)) {
    qxn->y = qyn1->y;
    n = n + 1;
  } else {
    qyn1->y = qxn->y;
    n = 0;
  }

  if (abs(qxn->z - qyn1->z) >= T && (n < c)) {
    qxn->z = qyn1->z;
    n = n + 1;
  } else {
    qyn1->z = qxn->z;
    n = 0;
  }

  return;
}
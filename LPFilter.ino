Quaternion LPFilter(Quaternion* qxn, Quaternion* qxn1, Quaternion* qyn1) {
  /*This function's purpose is to terminate high frequent outbreaks and further
  smooth the acquired sensor data
    qyn  -- Filtered sensor data to compute
    qyn1 -- previous computed filtered sensor data
    qxn  -- current raw measured sensor data
    qxn1 -- previous measured raw sensor data
  */
  //=======================================================
  //======            FUNCTION Variables            =======
  //=======================================================
    // Empty container for filtered quaterion
  Quaternion qyn;
  
  // coefficients for constant coefficient differential equation
  float a1 = 0;
  float b0 = 0;
  float b1 = 0;

  // Apply filter on extracted measured data
  qyn.w = a1 * qyn1->w + b0 * qxn->w + b1 * qxn1->w;
  qyn.x = a1 * qyn1->x + b0 * qxn->x + b1 * qxn1->x;
  qyn.y = a1 * qyn1->y + b0 * qxn->y + b1 * qxn1->y;
  qyn.z = a1 * qyn1->z + b0 * qxn->z + b1 * qxn1->z;

  // Update containers for previous raw and filtered quaternions
  *qxn1 = *qxn;
  *qyn1 = qyn;
  
  return qyn;
}
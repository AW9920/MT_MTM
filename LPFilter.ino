Quaternion LPFilter(Quaternion* qxn, Quaternion* qxn1, Quaternion* qyn1) {
  /*This function's purpose is to terminate high frequent outbreaks and further
    smooth the acquired sensor data
    qyn  -- Filtered sensor data to compute
    qyn1 -- previous computed filtered sensor data
    qxn  -- current raw measured sensor data
    qxn1 -- previous measured raw sensor data

    NOTE: The coefficients for Test2 and Test3 are looking the best. 
    Best results might lie in between both.
  */
  //=======================================================
  //======            FUNCTION Variables            =======
  //=======================================================
  Quaternion qyn;      //Empty container for filtered quaterion
  bool enable = false;  //Enabling LP Filter (Debugging)
  // coefficients for constant coefficient differential equation
  float a1 = 0.8945;
  float b0 = 0.0528;
  float b1 = 0.0528;

  // Apply filter on extracted measured data
  if (enable == true) {
    qyn.w = a1 * qyn1->w + b0 * qxn->w + b1 * qxn1->w;
    qyn.x = a1 * qyn1->x + b0 * qxn->x + b1 * qxn1->x;
    qyn.y = a1 * qyn1->y + b0 * qxn->y + b1 * qxn1->y;
    qyn.z = a1 * qyn1->z + b0 * qxn->z + b1 * qxn1->z;

    // Update containers for previous raw and filtered quaternions
    /**qxn1 = *qxn;
    *qyn1 = qyn;*/

  } else {
    //Return unfiltered values
    qyn = *qxn;
  }

  return qyn;
}
/* Test 1
    float a1 = 0.7285;
    float b0 = 0.1358;
    float b1 = 0.1358;
  */
//*  Test 2 (fg = 100Hz)
// float a1 = 0.8945;
// float b0 = 0.0528;
// float b1 = 0.0528;

/*Test 3 (fg)
  float a1 = 0.9458;
  float b0 = 0.0271;
  float b1 = 0.0271;
  */
/* Test 4
    float a1 = 0.978;
    float b0 = 0.011;
    float b1 = 0.011;
  */

/* Test 5
    float a1 = 0.5218;
    float b0 = 0.2391;
    float b1 = 0.2391;
  */

//Test 6 (fg=14.286 Hz)
/*
    float a1 = 0.906;
    float b0 = 0.047;
    float b1 = 0.047;
*/
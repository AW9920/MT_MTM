float LPFilter_Encoder(unsigned int* xn, unsigned int* xn1, unsigned int* yn1) {
  /*This function's purpose is to terminate high frequent outbreaks and further
    smooth the acquired sensor data
    yn  -- Filtered sensor data to compute
    yn1 -- previous computed filtered sensor data
    xn  -- current raw measured sensor data
    xn1 -- previous measured raw sensor data

    NOTE: The coefficients for Test2 and Test3 are looking the best. 
    Best results might lie in between both.
  */
  //=======================================================
  //======            FUNCTION Variables            =======
  //=======================================================
  float yn;            //Empty container for filtered quaterion
  bool enable = true;  //Enabling LP Filter (Debugging)
                       // coefficients for constant coefficient differential equation
  float a1 = 0.5218;
  float b0 = 0.2391;
  float b1 = 0.2391;

  // Apply filter on extracted measured data
  if (enable == true) {
    yn = a1 * (float)*yn1 + b0 * (float)*xn + b1 * (float)*xn1;

    /* Update containers for previous raw and filtered quaternions
    *xn1 = *xn;
    *yn1 = yn;*/

  } else {
    //Return unfiltered values
    yn = *xn;
  }

  return yn;
}
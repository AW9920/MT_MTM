float LPFilter_Encoder(unsigned int* xn, unsigned int* xn1, unsigned int* yn1, bool* rollover, bool* rollunder) {
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
  float yn = 0;  //Empty container for filtered quaterion
  //bool enable = true;  //Enabling LP Filter (Debugging)
  // coefficients for constant coefficient differential equation
  float a1 = 0.5218;
  float b0 = 0.2391;
  float b1 = 0.2391;

  // Apply filter on extracted measured data
  if (*rollover == false && *rollunder == false) {
    yn = a1 * (float)*yn1 + b0 * (float)*xn + b1 * (float)*xn1;

  } else if (*rollover == true && *rollunder == false) {  //Case Rollover: Shortly deactivate filter and update filter variables
    //Return unfiltered values
    yn = *xn;
    *rollover = false;
    //Serial.println("Filter temporarily turned off");

  } else if (*rollover == false && *rollunder == true) {
    yn = *xn;
    *rollunder = false;
    //Serial.println("Filter temporarily turned off");
  }

  return yn;
}
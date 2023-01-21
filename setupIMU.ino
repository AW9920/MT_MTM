void setupIMU(unsigned int AD, int i) {
  //Initialize IMU Device
  // initialize Device
  mpu[i].initialize();

  //Serial.println(F("Testing device connections..."));
  //Serial.print("MPU6050 ");
  //Serial.print(i);
  //Serial.print(" ");
  //Serial.println(mpu[i].testConnection() ? F("connection successful") : F("connection failed"));
  //Load and initialize the DMP
  devStatus = mpu[i].dmpInitialize();

  if (devStatus == 0) {
    //Adjust Offset Values
    /*if (AD == ADR) {
      mpu[i].setXAccelOffset(MPU6050R_ACCEL_OFFSET_X);
      mpu[i].setYAccelOffset(MPU6050R_ACCEL_OFFSET_Y);
      mpu[i].setZAccelOffset(MPU6050R_ACCEL_OFFSET_Z);
      mpu[i].setXGyroOffset(MPU6050R_GYRO_OFFSET_X);
      mpu[i].setYGyroOffset(MPU6050R_GYRO_OFFSET_Y);
      mpu[i].setZGyroOffset(MPU6050R_GYRO_OFFSET_Z);
    }

    else if (AD == ADL) {
      mpu[i].setXAccelOffset(MPU6050L_ACCEL_OFFSET_X);
      mpu[i].setYAccelOffset(MPU6050L_ACCEL_OFFSET_Y);
      mpu[i].setZAccelOffset(MPU6050L_ACCEL_OFFSET_Z);
      mpu[i].setXGyroOffset(MPU6050L_GYRO_OFFSET_X);
      mpu[i].setYGyroOffset(MPU6050L_GYRO_OFFSET_Y);
      mpu[i].setZGyroOffset(MPU6050L_GYRO_OFFSET_Z);
    }

    else {
      return;
    }*/
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu[i].CalibrateAccel(6);
    mpu[i].CalibrateGyro(6);
    mpu[i].PrintActiveOffsets();

    //Serial.println(F("Enabling DMP..."));
    mpu[i].setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu[i].dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
    //Serial.println("Error! Automatic device reset initialized!");
    //Cause Reset
    // digitalWrite(RESET, LOW);
    // delay(1);
    // digitalWrite(RESET, HIGH);
    // while (1)
  }
}
/*
if (!dmpReady) {
  Serial.println(dmpReady);
  Serial.println("IMU not connected.");
  delay(10);
  return;
}*/
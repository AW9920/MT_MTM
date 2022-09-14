void setupIMU(unsigned int AD) {
  //Initialize IMU Device
  // initialize Device
  mpu.initialize();
  //Load and initialize the DMP
  devStatus = mpu.dmpInitialize();

  //Adjust Offset Values
  if (AD == ADR) {
    mpu.setXAccelOffset(MPU6050R_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050R_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050R_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050R_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050R_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050R_GYRO_OFFSET_Z);
  }

  else if (AD == ADL) {
    mpu.setXAccelOffset(MPU6050L_ACCEL_OFFSET_X);
    mpu.setYAccelOffset(MPU6050L_ACCEL_OFFSET_Y);
    mpu.setZAccelOffset(MPU6050L_ACCEL_OFFSET_Z);
    mpu.setXGyroOffset(MPU6050L_GYRO_OFFSET_X);
    mpu.setYGyroOffset(MPU6050L_GYRO_OFFSET_Y);
    mpu.setZGyroOffset(MPU6050L_GYRO_OFFSET_Z);
  }

  else {
    return;
  }

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // Error
    Serial.println("Error! Automatic device reset initialized!");
    //Cause Reset
    digitalWrite(RESET, LOW);
    delay(1);
    digitalWrite(RESET, HIGH);
    while (1)
      ;
  }
}
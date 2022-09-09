void readIMU(Quaternion *q, int i)
{
  int  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024) { // check if overflow
    mpu.resetFIFO();
  } else if (mpuIntStatus & 0x02) {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    //Read Quaternions from FIFOBuffer and store in variable q
    mpu.dmpGetQuaternion(q, fifoBuffer);
    #ifdef DEBUGGING
    if (i == ADL) {
      Serial.print(q->w, 4); Serial.print("/");   //Delimiter "/" to distinguish values of individual
      Serial.print(q->x, 4); Serial.print("/");
      Serial.print(q->y, 4); Serial.print("/");
      Serial.print(q->z, 4); Serial.print("/");
      Serial.println();
    }
    #endif
  }
}
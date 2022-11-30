void Initial_preVal_def(void) {
  //-----------Read IMU data and define initial previous values--------------------
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    readIMU(qxn[i], i);
    *qsn[i] = spikeDetection(qxn[i], qsn1[i], dif[i],i);
    *qyn[i] = LPFilter(qsn[i], qxn1[i], qyn1[i]);
    UpdateQuat(qyn1[i], qyn[i]);  //Initial definition of previous filtered IMU measurement
    UpdateQuat(qxn1[i], qxn[i]);  //Initial definition of previous raw IMU measurement
  }

  //------------Read Encoder and define initial previous values--------------------
  for (unsigned int i = 0; i < (sizeof(EncDataR) / sizeof(EncDataR[0])); i++) {
    readEncoder(EncR_xn[i], DataPinR[i], CSR[i], CLKR, i);
    delayMicroseconds(1);
  }
  for (unsigned int i = 0; i < (sizeof(EncDataL) / sizeof(EncDataL[0])); i++) {
    readEncoder(EncL_xn[i], DataPinL[i], CSL[i], CLKL, i);
    delayMicroseconds(1);
  }
  for (unsigned int i = 0; i < (sizeof(EncDataR) / sizeof(EncDataR[0])); i++) {
    *EncR_yn[i] = LPFilter_Encoder(EncR_xn[i], EncR_xn1[i], EncR_yn1[i], rolloverR[i], rollunderR[i], i);
    updateArray(EncR_xn1[i], EncR_xn[i]);  //Initial definition of previous filtered Encoder RIGHT measurement
    updateArray(EncR_yn1[i], EncR_yn[i]);  //Initial definition of previous raw Encoder RIGHT measurement
  }
  for (unsigned int i = 0; i < (sizeof(EncDataL) / sizeof(EncDataL[0])); i++) {
    *EncL_yn[i] = LPFilter_Encoder(EncL_xn[i], EncL_xn1[i], EncL_yn1[i], rolloverL[i], rollunderL[i], i);
    updateArray(EncL_xn1[i], EncL_xn[i]);  //Initial definition of previous filtered Encoder LEFT measurement
    updateArray(EncL_yn1[i], EncL_yn[i]);  //Initial definition of previous raw Encoder LEFT measurement
  }
}
void SerialPrintData(int type) {
  //0 ->  "SamplingTime"
  //1 ->  "Quaternion for Monitor"
  //2 ->  "Encoder for Monitor"
  //3 ->  "Hall sensor data for Monitor"
  //4 ->  "All Data for Monitor"
  //5 ->  "SpikeDetection_ready2plot"
  //6 ->  "Processed encoder signals"
  //7 ->  "Evaluation of Spike detection"
  //8 ->  "Evaluation of LP filter IMU"
  //9 ->  "Evaluation of RolloverDetection"
  //10 ->  "Evaluation of LP filter Encoder"
  //11 ->  "Evaluation of RolloverDetection for Enc 2"
  //12 ->  "Evaluation of RolloverDetection left MTM"

  switch (type) {
    case 0:
      samplingTime = millis() - currentTime;
      //Output sampling Time
      Serial.print("Sampling Time:");
      Serial.println(samplingTime);
      break;

    case 1:
      // Quaternion right arm orientation
      Serial.print("quat right:\t");
      Serial.print(qR.w, 4);
      Serial.print("\t");
      Serial.print(qR.x, 4);
      Serial.print("\t");
      Serial.print(qR.y, 4);
      Serial.print("\t");
      Serial.print(qR.z, 4);
      Serial.print("\t");
      //Quaternion left arm orientation
      Serial.print("quat left:\t");
      Serial.print(qL.w, 4);
      Serial.print("\t");
      Serial.print(qL.x, 4);
      Serial.print("\t");
      Serial.print(qL.y, 4);
      Serial.print("\t");
      Serial.println(qL.z, 4);
      break;

    case 2:
      //Encoder right arm
      Serial.print("[ShoulderP, Elbow, ShoulderY] right:\t");
      Serial.print(Enc1R);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2R);  //Elbow
      Serial.print("\t");
      Serial.print(Enc3R);  //Shoulder Yaw
      Serial.print("\t");
      //Encoder left arm
      Serial.print("[ShoulderP, Elbow, ShoulderY] left:\t");
      Serial.print(Enc1L);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2L);  //Elbow
      Serial.print("\t");
      Serial.println(Enc3L);  //Shoulder Yaw
      break;

    case 3:
      //Hall right
      Serial.print("Hall sensor right:\t");
      Serial.print(HallR);
      Serial.print("\t");
      //Hall left
      Serial.print("Hall sensor left:\t");
      Serial.println(HallL);
      break;

    case 4:
      // Quaternion right arm orientation
      Serial.print("quat right:\t");
      Serial.print(qR.w, 4);
      Serial.print("\t");
      Serial.print(qR.x, 4);
      Serial.print("\t");
      Serial.print(qR.y, 4);
      Serial.print("\t");
      Serial.print(qR.z, 4);
      Serial.print("\t");
      //Quaternion left arm orientation
      Serial.print("quat left:\t");
      Serial.print(qL.w, 4);
      Serial.print("\t");
      Serial.print(qL.x, 4);
      Serial.print("\t");
      Serial.print(qL.y, 4);
      Serial.print("\t");
      Serial.print(qL.z, 4);
      Serial.print("\t");
      //Encoder right arm
      Serial.print(Enc1R);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2R);  //Elbow
      Serial.print("\t");
      Serial.print(Enc3R);  //Shoulder Yaw
      Serial.print("\t");
      //Encoder left arm
      Serial.print(Enc1L);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2L);  //Elbow
      Serial.print("\t");
      Serial.print(Enc3L);  //Shoulder Yaw
      Serial.print("\t");
      //Hall right
      Serial.print(HallR);
      Serial.print("\t");
      //Hall left
      Serial.println(HallL);
      break;

    case 5:
      //Difference between current and previous filtered values
      //Right
      Serial.print(dqR.w, 4);
      Serial.print("\t");
      Serial.print(dqR.x, 4);
      Serial.print("\t");
      Serial.print(dqR.y, 4);
      Serial.print("\t");
      Serial.print(dqR.z, 4);
      Serial.print("\t");
      //Left
      Serial.print(dqL.w, 4);
      Serial.print("\t");
      Serial.print(dqL.x, 4);
      Serial.print("\t");
      Serial.print(dqL.y, 4);
      Serial.print("\t");
      Serial.print(dqL.z, 4);
      Serial.println("\t");
      break;

    case 6:
      //Encoder right arm
      Serial.print("[ShoulderP, Elbow, ShoulderY] right:\t");
      Serial.print(Enc1R_inc);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2R_inc);  //Elbow
      Serial.print("\t");
      Serial.print(Enc3R_inc);  //Shoulder Yaw
      Serial.print("\t");
      //Encoder left arm
      Serial.print("[ShoulderP, Elbow, ShoulderY] left:\t");
      Serial.print(Enc1L_inc);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2L_inc);  //Elbow
      Serial.print("\t");
      Serial.println(Enc3L_inc);  //Shoulder Yaw
      break;

    case 7:
      //Right IMU; raw values
      Serial.print(qxn[0]->w, 4);
      Serial.print("\t");
      Serial.print(qxn[0]->x, 4);
      Serial.print("\t");
      Serial.print(qxn[0]->y, 4);
      Serial.print("\t");
      Serial.print(qxn[0]->z, 4);
      Serial.print("\t");
      //Right IMU; spike detected values
      Serial.print(qsn[0]->w, 4);
      Serial.print("\t");
      Serial.print(qsn[0]->x, 4);
      Serial.print("\t");
      Serial.print(qsn[0]->y, 4);
      Serial.print("\t");
      Serial.println(qsn[0]->z, 4);
      break;

    case 8:
      //Right IMU; raw values
      Serial.print(qxn[0]->w, 4);
      Serial.print("\t");
      Serial.print(qxn[0]->x, 4);
      Serial.print("\t");
      Serial.print(qxn[0]->y, 4);
      Serial.print("\t");
      Serial.print(qxn[0]->z, 4);
      Serial.print("\t");
      //Right IMU; spike detected and LP filtered values
      Serial.print(qyn[0]->w, 4);
      Serial.print("\t");
      Serial.print(qyn[0]->x, 4);
      Serial.print("\t");
      Serial.print(qyn[0]->y, 4);
      Serial.print("\t");
      Serial.println(qyn[0]->z, 4);
      break;

    case 9:
      // // Raw sensor values
      // Serial.print(*EncR_xn[0]);
      // Serial.print("\t");
      // Serial.print(*EncR_xn[1]);
      // Serial.print("\t");
      // Serial.print(*EncR_xn[2]);
      // Serial.print("\t");
      //Right encoders; raw values
      Serial.print(*EncR_yn[0] - EncR_OFF[0]);
      Serial.print("\t");
      Serial.print(*EncR_yn[1] - EncR_OFF[1]);
      Serial.print("\t");
      Serial.print(*EncR_yn[2] - EncR_OFF[2]);
      Serial.print("\t");
      //Right encoders; LP filtered and overflow detection
      Serial.print(Enc1R_inc);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2R_inc);  //Elbow
      Serial.print("\t");
      Serial.println(Enc3R_inc);  //Shoulder Yaw
      break;

    case 10:
      //Right encoders; raw values
      Serial.print(*EncR_xn[0]);
      Serial.print("\t");
      Serial.print(*EncR_xn[1]);
      Serial.print("\t");
      Serial.print(*EncR_xn[2]);
      Serial.print("\t");
      //Right encoders; LP filtered and overflow detection
      Serial.print(*EncR_yn[0]);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(*EncR_yn[1]);  //Elbow
      Serial.print("\t");
      Serial.println(*EncR_yn[2]);  //Shoulder Yaw
      break;

    case 11:
      Serial.print(*EncR_yn[1]);
      Serial.print("\t");
      Serial.println(Enc2R_inc);  //Elbow
      break;

    case 12:
      Serial.print(*EncL_yn[0] - EncL_OFF[0]);
      Serial.print("\t");
      Serial.print(*EncL_yn[1] - EncL_OFF[1]);
      Serial.print("\t");
      Serial.print(*EncL_yn[2] - EncL_OFF[2]);
      Serial.print("\t");
      //Right encoders; LP filtered and overflow detection
      Serial.print(Enc1L_inc);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2L_inc);  //Elbow
      Serial.print("\t");
      Serial.println(Enc3L_inc);  //Shoulder Yaw
      break;

      case 13: //Coodinates MTM / PSM
      Serial.print(C_mtm[0]);
      Serial.print("\t");
      Serial.print(C_mtm[1]);
      Serial.print("\t");
      Serial.print(C_mtm[2]);
      Serial.print("\t");
      Serial.print(C_psm[0]); 
      Serial.print("\t");
      Serial.print(C_psm[1]);
      Serial.print("\t");
      Serial.println(C_psm[2]);
      break;

      case 14:
      //Quaternion left arm orientation
      Serial.print("quat left:\t");
      Serial.print(qL.w, 4);
      Serial.print("\t");
      Serial.print(qL.x, 4);
      Serial.print("\t");
      Serial.print(qL.y, 4);
      Serial.print("\t");
      Serial.print(qL.z, 4);
      Serial.print("\t");
      //Encoder left arm
      Serial.print(Enc1L);  //Shoulder Pitch
      Serial.print("\t");
      Serial.print(Enc2L);  //Elbow
      Serial.print("\t");
      Serial.print(Enc3L);  //Shoulder Yaw
      Serial.print("\t");
      //Hall left
      Serial.println(HallL);
      break;

  }
}
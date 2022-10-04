void SerialPrintData(int type) {
  //0 ->  "SamplingTime"
  //1 ->  "Quaternion for Monitor"
  //2 ->  "Encoder for Monitor"
  //3 ->  "Hall sensor data for Monitor"
  //4 ->  "All Data for Monitor"
  //5 ->  "SpikeDetection_ready2plot"

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
      Serial.print(qL.z, 4);
      Serial.println("\t");
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
  }
}
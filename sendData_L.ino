void sendData_L(double q[7]){
  String buffer = "";
  buffer = compData(q[0], q[1], q[2], q[3], q[4], q[5], q[6], 2);
  //Check if enough bytes are available for writing
  int dataSize = buffer.length();
  if (Serial2.availableForWrite() >= dataSize) {
    Serial2.println(buffer);
    Serial.println(buffer);
    //Serial.println(buffer);
  } else {
    Serial.println("Serial2 buffer is full. Data not sent.");
  }
}
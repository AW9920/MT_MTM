void parseData(void) {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");  // get the first part - the string
  C_psm[0] = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");  // this continues where the previous call left off
  C_psm[1] = atof(strtokIndx);

  strtokIndx = strtok(NULL, ",");
  C_psm[2] = atof(strtokIndx);
}
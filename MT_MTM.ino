//Define Arduino UNO CPU clock
#define F_CPU 16000000L

//=======================================================
//======            Include libraries             =======
//=======================================================

//#include <avr/wdt.h>  //Watchdog Timer Library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdfix.h>
#include <math.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//=======================================================
//======     Define RUN, DEBUG, EVALUATION        =======
//=======================================================
//#define RUN
//#define DEBUG
#define EVAL

//=======================================================
//======                 Makros                   =======
//=======================================================
// Defina I2C address
#define ADR 0  //Order for Read out and Setup
#define ADL 1  //

//Define Pins
#define RESET 14  // Responsible for RESET
#define DO1R 2    // Encoder Shoulder Pitch right
#define DO2R 3    // Encoder Shoulder Yaw right
#define DO3R 4    // Encoder Elbow right
#define DO1L 5    // Encoder Shoulder Pitch left
#define DO2L 6    // Encoder Shoulder Yaw left
#define DO3L 7    // Encoder Elbow left

#define CS1R 8   // Chip Select right
#define CS2R 9   // %
#define CS3R 10  // %
#define CS1L 11  // Chip select left
#define CS2L 12  // %
#define CS3L 13  // %

#define AD0R 29  // IMU Address right
#define AD0L 30  // IMU Address left

#define CLKR 44  // Clock signal right
#define CLKL 46  // Clock signal left

#define HDOR 0  // Analog Input 0 (Hall Sensor)
#define HDOL 1

/* Old IMU Calc Data
  //Define Calibration Values for IMU right which yielded best results
  #define MPU6050_ACCEL_OFFSET_X -3954
  #define MPU6050_ACCEL_OFFSET_Y -4162
  #define MPU6050_ACCEL_OFFSET_Z 1546
  #define MPU6050_GYRO_OFFSET_X  84
  #define MPU6050_GYRO_OFFSET_Y  -28
  #define MPU6050_GYRO_OFFSET_Z  26
*/
//Define Calibration Values for IMU right which yielded best results
#define MPU6050R_ACCEL_OFFSET_X -3616
#define MPU6050R_ACCEL_OFFSET_Y -1092
#define MPU6050R_ACCEL_OFFSET_Z 902
#define MPU6050R_GYRO_OFFSET_X 52
#define MPU6050R_GYRO_OFFSET_Y -188
#define MPU6050R_GYRO_OFFSET_Z 32

//Define Calibration Values for IMU left which yielded best results
#define MPU6050L_ACCEL_OFFSET_X -788
#define MPU6050L_ACCEL_OFFSET_Y 1417
#define MPU6050L_ACCEL_OFFSET_Z 998
#define MPU6050L_GYRO_OFFSET_X 37
#define MPU6050L_GYRO_OFFSET_Y -14
#define MPU6050L_GYRO_OFFSET_Z 40


//=======================================================
//======             GLOBAL VARIABLES             =======
//=======================================================
/*--------Causes a Null Operation which has no effect-----*/
//#define NOP __asm__ __volatile__ ("nop\n\t")        //Skip one single tick. 1NOP = 1tick = 62.5ns
#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)  //Skips n ticks. delay = n*62.5ns /n is integer
unsigned int const delay_375ns = 6;
unsigned int const delay_500ns = 8;

/*-------------------MPU control/status vars--------------*/
bool dmpReady;  // set true if DMP init was successful
bool IMUready;
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

/*---------------------create IMU Object------------------*/
MPU6050 mpuR(0x68);
MPU6050 mpuL(0x69);
MPU6050 mpu[] = { mpuR, mpuL };

/*-------------------OFFSET Encoder values----------------*/
float const Enc1R_OFF = 1684; //1684;
float const Enc2R_OFF = 2350; //2353; new 2350
float const Enc3R_OFF = 1832; //1823; new 1832
float EncR_OFF[3] = { Enc1R_OFF, Enc2R_OFF, Enc3R_OFF };

float const Enc1L_OFF = 526;
float const Enc2L_OFF = 1355;
float const Enc3L_OFF = 3109;
float EncL_OFF[3] = { Enc1L_OFF, Enc2L_OFF, Enc3L_OFF };

/*--------------------OVERFLOW detection------------------*/
int temp_side = 0;
int const high_lim = 2047;
int const low_lim = -2048;
int const gain = 4095;
int countR1 = 0, countR2 = 0, countR3 = 0;
int *countR[3] = { &countR1, &countR2, &countR3 };
int countL1 = 0, countL2 = 0, countL3 = 0;
int *countL[3] = { &countL1, &countL2, &countL3 };
bool rolloverR1 = false, rolloverR2 = false, rolloverR3 = false;
bool rolloverL1 = false, rolloverL2 = false, rolloverL3 = false;
bool rollunderR1 = false, rollunderR2 = false, rollunderR3 = false;
bool rollunderL1 = false, rollunderL2 = false, rollunderL3 = false;
bool *rolloverR[3] = { &rolloverR1, &rolloverR2, &rolloverR3 };
bool *rolloverL[3] = { &rolloverL1, &rolloverL2, &rolloverL3 };
bool *rollunderR[3] = { &rollunderR1, &rollunderR2, &rollunderR3 };
bool *rollunderL[3] = { &rollunderL1, &rollunderL2, &rollunderL3 };

int Enc1R_inc, Enc2R_inc, Enc3R_inc;
int Enc1L_inc, Enc2L_inc, Enc3L_inc;
int *EncDataR_inc[3] = { &Enc1R_inc, &Enc2R_inc, &Enc3R_inc };
int *EncDataL_inc[3] = { &Enc1L_inc, &Enc2L_inc, &Enc3L_inc };

/*-----------------------Orientation and motion vars-------------------------*/
//Quaternion array --> q = [w, x, y, z]
Quaternion qR, qL;  //current raw quaternion container; left / right IMU
Quaternion *q[2] = { &qR, &qL };

/*----------------------Variables for Low-Pass Filter-------------------------*/
//Quaternion Data
Quaternion qxnR, qxnL;    //temp raw value; left / right IMU
Quaternion qxn1R, qxn1L;  //previous raw value; left / right IMU
Quaternion qyn1R, qyn1L;  //previous filtered value; left / right IMU
Quaternion qynR, qynL;    //previous filtered value; left / right IMU
Quaternion *qxn[2] = { &qxnR, &qxnL };
Quaternion *qxn1[2] = { &qxn1R, &qxn1L };
Quaternion *qyn1[2] = { &qyn1R, &qyn1L };
Quaternion *qyn[2] = { &qynR, &qynL };
//Encoder Data
unsigned int Enc1R_xn, Enc2R_xn, Enc3R_xn;     //Temp raw measurement of encoders RIGHT
unsigned int Enc1R_xn1, Enc2R_xn1, Enc3R_xn1;  //Last raw measurement of encoders RIGHT
unsigned int Enc1R_yn1, Enc2R_yn1, Enc3R_yn1;  //Last filtered measurement of encoders RIGHT
unsigned int Enc1R_yn, Enc2R_yn, Enc3R_yn;     //Last filtered measurement of encoders RIGHT
unsigned int Enc1L_xn, Enc2L_xn, Enc3L_xn;     //Last raw measurement of encoders LEFT
unsigned int Enc1L_xn1, Enc2L_xn1, Enc3L_xn1;  //Last raw measurement of encoders LEFT
unsigned int Enc1L_yn1, Enc2L_yn1, Enc3L_yn1;  //Last filtered measurement of encoders LEFT
unsigned int Enc1L_yn, Enc2L_yn, Enc3L_yn;     //Temp filtered measurement of encoders LEFT
unsigned int *EncR_xn[3] = { &Enc1R_xn, &Enc2R_xn, &Enc3R_xn };
unsigned int *EncR_xn1[3] = { &Enc1R_xn1, &Enc2R_xn1, &Enc3R_xn1 };
unsigned int *EncR_yn1[3] = { &Enc1R_yn1, &Enc2R_yn1, &Enc3R_yn1 };
unsigned int *EncR_yn[3] = { &Enc1R_yn, &Enc2R_yn, &Enc3R_yn };
unsigned int *EncL_xn[3] = { &Enc1L_xn, &Enc2L_xn, &Enc3L_xn };
unsigned int *EncL_xn1[3] = { &Enc1L_xn1, &Enc2L_xn1, &Enc3L_xn1 };
unsigned int *EncL_yn1[3] = { &Enc1L_yn1, &Enc2L_yn1, &Enc3L_yn1 };
unsigned int *EncL_yn[3] = { &Enc1L_yn, &Enc2L_yn, &Enc3L_yn };

/*----------------------Variables for spike detection------------------------*/
Quaternion qsn1R, qsn1L;  //previous safe value; left / right IMU
Quaternion qsnR, qsnL;    //current Filter output (Debugging)
Quaternion *qsn1[2] = { &qsn1R, &qsn1L };
Quaternion *qsn[2] = { &qsnR, &qsnL };  //(Debugging)
Quaternion dqR, dqL;
Quaternion dq[2] = { dqR, dqL };
int cwr, cxr, cyr, czr;
int cwl, cxl, cyl, czl;
int cr[4] = { cwr, cxr, cyr, czr };  //subsequent spike counter
int cl[4] = { cwl, cxl, cyl, czl };  //subsequent spike counter
int *c[2] = { cr, cl };

float dif_R[4], dif_L[4];
float *dif[2] = { dif_R, dif_L };

/*--------------------------Variables for Encoder data-----------------------*/
int Enc1R, Enc2R, Enc3R;                          //Assigne Variable to Memory
int Enc1L, Enc2L, Enc3L;                          //Assigne Variable to Memory
int *EncDataR[3] = { &Enc1R, &Enc2R, &Enc3R };    //Pointer Array Right Encoders declaration
int *EncDataL[3] = { &Enc1L, &Enc2L, &Enc3L };    //Pointer Array Left Encoders declaration
unsigned int DataPinR[3] = { DO1R, DO2R, DO3R };  //Data Pins Encoder right arm
unsigned int DataPinL[3] = { DO1L, DO2L, DO3L };  //Data Pins Encoder left arm
unsigned int CSR[3] = { CS1R, CS2R, CS3R };       //Pointer Array Chip selection Pin Right
unsigned int CSL[3] = { CS1L, CS2L, CS3L };       //Pointer Array Chip selection Pin Left
unsigned int ADDR[2] = {
  ADR,  //Right IMU index 0
  ADL   //Left IMU index 1
};

/*---------------------------Hall Sensor variables---------------------------*/
int HallR;
int HallL;

/*----------------------------Other variables--------------------------------*/
unsigned long currentTime;
unsigned long samplingTime;

// Encoder control/status vars. Unity calculates true angle
/*int bit_res = 4096, FSR = 360;  //FSR = Full-Range-Scale    bit_res = Data Bits
  float angle_res = (float)FSR / (float)bit_res;*/

//=======================================================
//======            FUNCTION DECLARATION          =======
//=======================================================
//Function declaration //To ease debugging
void setupIMU(unsigned int AD);
void readEncoder(unsigned int *OutData, unsigned int DO, int CSn, unsigned int CLK, int i);
void readIMU(Quaternion *q, int i);
Quaternion LPFilter(Quaternion *qxn, Quaternion *qxn1, Quaternion *qyn1);
void spikeDetection(Quaternion *qxn, Quaternion *qyn1);
void UpdateQwF(Quaternion *q, float *q_val);
float *Quat2floatArr(Quaternion *q);
void updateArray(float *arr1, float *arr2);
void breakpoint(void);
void WarmUpIMU(void);
void sendData(void);

//=======================================================
//======              INITIAL SETUP               =======
//=======================================================
void setup() {
  //Initialize PINS
  pinMode(RESET, OUTPUT);
  //Chip Select Encoders
  pinMode(CS1R, OUTPUT);
  pinMode(CS2R, OUTPUT);
  pinMode(CS3R, OUTPUT);
  pinMode(CS1L, OUTPUT);
  pinMode(CS2L, OUTPUT);
  pinMode(CS3L, OUTPUT);
  //Clock Encoders
  pinMode(CLKR, OUTPUT);
  pinMode(CLKL, OUTPUT);
  //Data Encoders
  pinMode(DO1R, INPUT);
  pinMode(DO2R, INPUT);
  pinMode(DO3R, INPUT);
  pinMode(DO1L, INPUT);
  pinMode(DO2L, INPUT);
  pinMode(DO3L, INPUT);
  //I2C Address Set
  pinMode(AD0R, OUTPUT);
  pinMode(AD0L, OUTPUT);
  //Define Pin Output
  digitalWrite(RESET, HIGH);
  digitalWrite(CLKR, HIGH);
  digitalWrite(CLKL, HIGH);
  digitalWrite(CS1R, HIGH);
  digitalWrite(CS2R, HIGH);
  digitalWrite(CS3R, HIGH);
  digitalWrite(CS1L, HIGH);
  digitalWrite(CS2L, HIGH);
  digitalWrite(CS3L, HIGH);
  //Set addresses of IMUs
  digitalWrite(AD0R, LOW);   //I2C address 0x68
  digitalWrite(AD0L, HIGH);  //I2C Address 0x69

  //Setup Serial Communication
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  //Set Baudrate at 115200 Bps
  Serial.begin(115200);
  while (!Serial) {};  // wait for Leonardo enumeration, others continue immediately

  //Setup IMUs
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    //Serial.println(ADDR[i]);
    setupIMU(ADDR[i], i);
  }

  //Initial definition of previous values
  Serial.println("Initializing previous value variables!");
  Initial_preVal_def();


  Serial.println("Setup successful!");
  delay(200);
}

//=======================================================
//======              MAIN ROUTINE                =======
//=======================================================
void loop() {
  // Update current Time at begin of sensor data sampling
  currentTime = millis();

  //---------------------Get Quaternion Data-------------------------------
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    //Acquire Data from IMU sensor
    readIMU(qxn[i], i);

    //Spike Filter
    *qsn[i] = spikeDetection(qxn[i], qsn1[i], dif[i],i);

    //Filtering IMU data; Terminate outbreaks
    *qyn[i] = LPFilter(qsn[i], qxn1[i], qyn1[i]);

    //Cap the quaternions received
    qyn[i]->w = constrain(qyn[i]->w, -1, 1);
    qyn[i]->x = constrain(qyn[i]->x, -1, 1);
    qyn[i]->y = constrain(qyn[i]->y, -1, 1);
    qyn[i]->z = constrain(qyn[i]->z, -1, 1);

    //Update values
    UpdateQuat(qyn1[i], qyn[i]);  //Update previous filtered IMU measurement
    UpdateQuat(qxn1[i], qxn[i]);  //Update previous raw IMU measurement
    //UpdateQuat(qsn1, qsn);  //Update previous save Spike Detection value (already done inside function)

    //Pack processed data into variables send over Serial Bus
    UpdateQuat(q[i], qyn[i]);
  }

  //-----------------------Get Encoder Data Right-------------------------------
  for (unsigned int i = 0; i < (sizeof(EncDataR) / sizeof(EncDataR[0])); i++) {
    readEncoder(EncR_xn[i], DataPinR[i], CSR[i], CLKR, i);  //Hand over Memory address of EncData and overwrite values
    delayMicroseconds(1);                                   //Tcs waiting for another read in
  }

  //------------------------Get Encoder Data Left-------------------------------
  for (unsigned int i = 0; i < (sizeof(EncDataL) / sizeof(EncDataL[0])); i++) {
    readEncoder(EncL_xn[i], DataPinL[i], CSL[i], CLKL, i);  //Hand over Memory address of EncData and overwrite values
    delayMicroseconds(1);                                   //Tcs waiting for another read in
  }

  //--------------Digitla low pas filter on encoder data-------------------------
  for (unsigned int i = 0; i < (sizeof(EncDataR) / sizeof(EncDataR[0])); i++) {
    //----Overlow detection----
    OverFlowDetection(EncR_xn[i], EncR_xn1[i], rolloverR[i], rollunderR[i], countR[i]);

    //-------LP filter---------
    *EncR_yn[i] = LPFilter_Encoder(EncR_xn[i], EncR_xn1[i], EncR_yn1[i], rolloverR[i], rollunderR[i]);

    //Update values
    updateArray(EncR_xn1[i], EncR_xn[i]);
    updateArray(EncR_yn1[i], EncR_yn[i]);

    //Pack processed data into variables send over Serial Bus
    *EncDataR_inc[i] = *EncR_yn[i] + *countR[i] * gain - EncR_OFF[i];
    updateArray(EncDataR[i], EncDataR_inc[i]);
  }
  for (unsigned int i = 0; i < (sizeof(EncDataL) / sizeof(EncDataL[0])); i++) {
    //----Overlow detection----
    OverFlowDetection(EncL_xn[i], EncL_xn1[i], rolloverL[i], rollunderL[i], countL[i]);

    //-------LP filter---------
    *EncL_yn[i] = LPFilter_Encoder(EncL_xn[i], EncL_xn1[i], EncL_yn1[i], rolloverL[i], rollunderL[i]);

    //Update values
    updateArray(EncL_xn1[i], EncL_xn[i]);
    updateArray(EncL_yn1[i], EncL_yn[i]);

    //Pack processed data into variables send over Serial Bus
    *EncDataL_inc[i] = *EncL_yn[i] + *countL[i] * gain - EncL_OFF[i];
    updateArray(EncDataL[i], EncDataL_inc[i]);
  }

  //Get Hall Sensor data
  HallR = analogRead(HDOR);
  HallL = analogRead(HDOL);

#ifdef RUN
  //Call Data Send function
  sendData();
#endif

#ifdef EVAL
  SerialPrintData(12);
#endif
}

//=======================================================
//======               Functions                  =======
//=======================================================

void sendData(void) {
  // Right Arm Motion Data
  Serial.print("r");
  Serial.print("/");  //ID right arm
  Serial.print(qR.w, 4);
  Serial.print("/");  //Delimiter "/" to distinguish values of individual
  Serial.print(qR.x, 4);
  Serial.print("/");
  Serial.print(qR.y, 4);
  Serial.print("/");
  Serial.print(qR.z, 4);
  Serial.print("/");
  Serial.print(Enc1R);
  Serial.print("/");  //Shoulder Pitch
  Serial.print(Enc2R);
  Serial.print("/");  //Elbow
  Serial.print(Enc3R);
  Serial.print("/");  //Shoulder Yaw
  Serial.print(HallR);
  Serial.print(";");  //Delimiter ";" to distinguish left and right arm

  // Left Arm Motion Data
  Serial.print("l");
  Serial.print("/");  //ID left arm
  Serial.print(qL.w, 4);
  Serial.print("/");  //Delimiter "/" to distinguish values of individual
  Serial.print(qL.x, 4);
  Serial.print("/");
  Serial.print(qL.y, 4);
  Serial.print("/");
  Serial.print(qL.z, 4);
  Serial.print("/");
  Serial.print(Enc1L);
  Serial.print("/");  //Shoulder Pitch
  Serial.print(Enc2L);
  Serial.print("/");  //Elbow
  Serial.print(Enc3L);
  Serial.print("/");      //Shoulder Yaw
  Serial.println(HallL);  //Delimiter ";" to distinguish left and right arm
}

void sendData2(void) {
  // Right Arm Motion Data
  Serial.print("r");
  Serial.print("/");  //ID right arm
  Serial.print(qR.w, 4);
  Serial.print("/");  //Delimiter "/" to distinguish values of individual
  Serial.print(qR.x, 4);
  Serial.print("/");
  Serial.print(qR.y, 4);
  Serial.print("/");
  Serial.print(qR.z, 4);
  Serial.print("/");
  Serial.print(Enc1R_inc);
  Serial.print("/");  //Shoulder Pitch
  Serial.print(Enc2R_inc);
  Serial.print("/");  //Elbow
  Serial.print(Enc3R_inc);
  Serial.print("/");  //Shoulder Yaw
  Serial.print(HallR);
  Serial.print(";");  //Delimiter ";" to distinguish left and right arm

  // Left Arm Motion Data
  Serial.print("l");
  Serial.print("/");  //ID left arm
  Serial.print(qL.w, 4);
  Serial.print("/");  //Delimiter "/" to distinguish values of individual
  Serial.print(qL.x, 4);
  Serial.print("/");
  Serial.print(qL.y, 4);
  Serial.print("/");
  Serial.print(qL.z, 4);
  Serial.print("/");
  Serial.print(Enc1L_inc);
  Serial.print("/");  //Shoulder Pitch
  Serial.print(Enc2L_inc);
  Serial.print("/");  //Elbow
  Serial.print(Enc3L_inc);
  Serial.print("/");      //Shoulder Yaw
  Serial.println(HallL);  //Delimiter ";" to distinguish left and right arm
}

void UpdateQuat(Quaternion *q_new, Quaternion *q_old) {
  q_new->w = q_old->w;
  q_new->x = q_old->x;
  q_new->y = q_old->y;
  q_new->z = q_old->z;
  return;
}

void UpdateQwF(Quaternion *q, float *q_val) {
  q->w = q_val[0];
  q->x = q_val[1];
  q->y = q_val[2];
  q->z = q_val[3];
  return;
}

float *Quat2floatArr(Quaternion *q) {
  static float Q_f[4];
  Q_f[0] = q->w;
  Q_f[1] = q->x;
  Q_f[2] = q->y;
  Q_f[3] = q->z;

  return Q_f;
}

void updateArray(float *arr1, float *arr2) {
  for (int i = 0; i < (sizeof(arr1) / sizeof(arr1[0])); i++) {
    // Serial.println(sizeof(arr1) / sizeof(arr1[0]));
    arr1[i] = arr2[i];
  }
}

void updateArray(unsigned int *arr1, unsigned int *arr2) {
  for (int i = 0; i < (sizeof(arr1) / sizeof(arr1[0])); i++) {
    //Serial.println(sizeof(arr1) / sizeof(arr1[0]));
    arr1[i] = arr2[i];
  }
}

void updateArray(int *arr1, int *arr2) {
  for (int i = 0; i < (sizeof(arr1) / sizeof(arr1[0])); i++) {
    //Serial.println(sizeof(arr1) / sizeof(arr1[0]));
    arr1[i] = arr2[i];
  }
}

void breakpoint(void) {
  Serial.println();
  Serial.print("Debugger Breakpoint! Enter any character to continue!");
  while (Serial.available() && Serial.read()) {};  // empty buffer
  while (!Serial.available()) {};                  // wait for data
  while (Serial.available() && Serial.read()) {};  // empty buffer again
}

void serialFlush() {
  Serial.flush();
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}
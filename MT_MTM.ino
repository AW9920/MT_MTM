//Define Arduino UNO CPU clock
#define F_CPU 16000000L

//=======================================================
//======            Include libraries             =======
//=======================================================

//#include <avr/wdt.h>  //Watchdog Timer Library
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h"
#include <helper_3dmath.h>
#include <stdfix.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <string.h>


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
//======    Define DMP or Complement filter       =======
//=======================================================
#define DMP
//#define COMP

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

#define MPU_ADDR_R 0x68
#define MPU_ADDR_L 0x69

//Define Calibration Values for IMU left in horizontal position
#define MPU6050L_ACCEL_OFFSET_X -788
#define MPU6050L_ACCEL_OFFSET_Y 1417
#define MPU6050L_ACCEL_OFFSET_Z 998
#define MPU6050L_GYRO_OFFSET_X 37
#define MPU6050L_GYRO_OFFSET_Y -14
#define MPU6050L_GYRO_OFFSET_Z 40

// -------------------------Calibration in horizontal-----------------------------
//Define Calibration Values for IMU right in horizontal position
#define MPU6050R_ACCEL_OFFSET_X -818
#define MPU6050R_ACCEL_OFFSET_Y 1453
#define MPU6050R_ACCEL_OFFSET_Z 992
#define MPU6050R_GYRO_OFFSET_X 33
#define MPU6050R_GYRO_OFFSET_Y -12
#define MPU6050R_GYRO_OFFSET_Z 37


// Math constants
#define PI 3.1415926535897932384626433832795

//=======================================================
//======             GLOBAL VARIABLES             =======
//=======================================================
/*--------Causes a Null Operation which has no effect-----*/
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

/*-------------------Serial Communication-----------------*/
//SoftwareSerial SUART(0, 1);

/*-------------------OFFSET Encoder values----------------*/
float const Enc1R_OFF = 1684;  //1684;
float const Enc2R_OFF = 2350;  //2353; new 2350
float const Enc3R_OFF = 1790;  //1823; new 1832
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
Quaternion pR = { 0.7044, -0.0001, -0.7098, -0.0027 }, pL = { 1, 0, 0, 0 };
Quaternion p[2] = { pR, pL };

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

/*----------------------------Kinematic Variables-----------------------------*/
//MTM
double q1_m = 0, q2_m = 0, q3_m = 0, q4_m = 0, q5_m = 0, q6_m = 0, q7_m = 0;
double q_mtm[7] = { q1_m, q2_m, q3_m, q4_m, q5_m, q6_m, q7_m };
double x_m, y_m, z_m;
double C_mtm[3] = { x_m, y_m, z_m };
const double x_m_init = 262.5, y_m_init = -217.5, z_m_init = 8.7;
double C_mtm_init[3] = { x_m_init, y_m_init, z_m_init };
//PSM
double q1_p = 0, q2_p = 0, q3_p = 0, q4_p = 0, q5_p = 0, q6_p = 0, q7_p = 0;
double q_psm[7] = { q1_p, q2_p, q3_p, q4_p, q5_p, q6_p, q7_p };
double x_p, y_p, z_p;
double C_psm[3] = { x_p, y_p, z_p };
const double x_p_init = 0, y_p_init = -56.51, z_p_init = 0;
double C_psm_init[3] = { x_p_init, y_p_init, z_p_init };

//General variables
const double motion_scaler_x = 0.05;
const double motion_scaler_y = 0.25;
const double motion_scaler_z = 0.05;
const double res_mag_enc = 0.0879;
const double L1 = 45, L2 = 217.5, L3 = 8.7, H1 = 217.5, d0 = -23.49;

//Wrist rotation variables
double quatArray[4];
double rotMat[9];
double mat[3][3];
double result[3][3];
double mat1[3][3] = { { -1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, -1.0 } };

/*---------------------------Hall Sensor variables---------------------------*/
int HallR;
int HallL;

/*----------------------------Other variables--------------------------------*/
unsigned long currentTime;
unsigned long samplingTime;
bool sys_ready = false;

/*-------------------------Receive Data variable-----------------------------*/
const byte numChars = 32;  //32
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
//variables to temp hold the parsed data
char messageFromPC[numChars] = { 0 };
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;

/*----------------------Estimation by Accel & Gyro----------------------------*/
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;  // Acceleration, temperature and angular velocity for each axis

double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFilX, angleFilY, angleFilZ;  // filtered angles
const double RADIAN_TO_DEGREE = 180 / 3.14159;
const double DEG_PER_SEC = 32767 / 250;  // rotating angle per second 250 degree // GyX, GyY, GyZ range  : -32768 ~ +32767
const double ALPHA = 0.65;               // 0.96 //0.6

unsigned long now = 0;   // current time
unsigned long past = 0;  // previous time
double dt = 0;

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;

/*--------------------------------Latency------------------------------------*/
unsigned long send_time;

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
  while (!Serial) {};
  Serial2.begin(115200);

#ifdef DMP
  //Setup IMUs
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    //Serial.println(ADDR[i]);
    setupIMU(ADDR[i], i);
  }
  //Initial definition of previous values
  Serial.println("Initializing previous value variables!");
  Initial_preVal_def();
  sys_ready = true;
  delay(200);
#endif

#if COMP
  initSensor();
  Initcalib();
  Serial.begin(115200);
  sys_ready = true;
  delay(200);
  past = millis();  // save the current time in past
#endif

  Serial.println("Setup successful!");
}

//=======================================================
//======              MAIN ROUTINE                =======
//=======================================================
void loop() {
  // Update current Time at begin of sensor data sampling
  currentTime = millis();

#ifdef DMP
  //---------------------Get Quaternion Data-------------------------------
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    //Acquire Data from IMU sensor
    readIMU(qxn[i], i);

    //Spike Filter
    *qsn[i] = spikeDetection(qxn[i], qsn1[i], dif[i], i);

    //Filtering IMU data; Terminate outbreaks
    *qyn[i] = LPFilter(qsn[i], qxn1[i], qyn1[i]);

    //Rotate along offset;
    Quaternion t;
    t = p[i].getProduct(qyn[i]->getConjugate()).getConjugate();
    *qyn[i] = t;
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
#endif

#ifdef COMP
  readRawData();
  getDT();

  //////////// angles from Accelration
  angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2)));
  angleAcX *= RADIAN_TO_DEGREE;
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2)));
  angleAcY *= RADIAN_TO_DEGREE;

  //////////// angles from Gyroscope
  angleGyX += ((GyX - averGyX) / DEG_PER_SEC) * dt;
  angleGyY += ((GyY - averGyY) / DEG_PER_SEC) * dt;
  angleGyZ += ((GyZ - averGyZ) / DEG_PER_SEC) * dt;

  double angleTmpX = angleFilX + angleGyX * dt;
  double angleTmpY = angleFilY + angleGyY * dt;
  double angleTmpZ = angleFilZ + angleGyZ * dt;
  angleFilX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
  angleFilY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
  angleFilZ = angleGyZ;  //
#endif

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

  //---------------------------Get Hall Sensor data-----------------------------------
  HallR = analogRead(HDOR);
  HallL = analogRead(HDOL);

  //-----------------------Convert to real angle values-------------------------------
  q_mtm[0] = double(*EncDataR[0]) * res_mag_enc * PI / 180.0;   //Convert 2 rad
  q_mtm[1] = -double(*EncDataR[2]) * res_mag_enc * PI / 180.0;  //Convert 2 rad
  q_mtm[2] = -double(*EncDataR[1]) * res_mag_enc * PI / 180.0;  //Convert 2 rad

  //-----------------------Compute planned trajectory---------------------------------
  // To-Do: Compute desired position of PSM in task space
  x_m = L1 * cos(q_mtm[0]) + L2 * cos(q_mtm[0]) * cos(q_mtm[1]) - L3 * cos(q_mtm[0]) * sin(q_mtm[1]) - H1 * sin(q_mtm[0]) * sin(q_mtm[2] - PI / 2) + H1 * cos(q_mtm[0]) * cos(q_mtm[1]) * cos(q_mtm[2] - PI / 2);
  y_m = L1 * sin(q_mtm[0]) + L2 * cos(q_mtm[1]) * sin(q_mtm[0]) + H1 * cos(q_mtm[0]) * sin(q_mtm[2] - PI / 2) - L3 * sin(q_mtm[0]) * sin(q_mtm[1]) + H1 * cos(q_mtm[1]) * cos(q_mtm[2] - PI / 2) * sin(q_mtm[0]);
  z_m = L3 * cos(q_mtm[1]) + L2 * sin(q_mtm[1]) + H1 * cos(q_mtm[2] - PI / 2) * sin(q_mtm[1]);
  C_mtm[0] = x_m;
  C_mtm[1] = y_m;
  C_mtm[2] = z_m;

  //Compute trajectory
  double d_x = x_m - x_m_init;
  double d_y = y_m - y_m_init;
  double d_z = z_m - z_m_init;
  x_p = x_p_init + motion_scaler_x * d_x;
  y_p = y_p_init + motion_scaler_y * d_y;
  z_p = z_p_init + motion_scaler_z * d_z;

  //-------------------Compute desired joint values 1, 2 & 3 for PSM---------------------------
  // To-Do: Conduct inverse Kinematics
  int sign = 0;
  if (y_p < 0) {
    sign = 1;
  } else if (y_p >= 0) {
    sign = -1;
  }
  //Compute desired q3 of PSM
  q3_p = sign * sqrt(sq(x_p) + sq(y_p) + sq(z_p)) - d0;
  //Compute desired q2 of PSM
  double s2 = -x_p / (d0 + q3_p);
  double c2 = sqrt(1 - sq(s2));
  q2_p = atan2(s2, c2);
  if ((q2_p <= (-PI / 2)) || (q2_p >= (PI / 2))) {
    q2_p = atan2(s2, -c2);
  }
  q2_p *= (180 / PI);
  //Compute desired q1 of PSM
  double b = c2 * (q3_p + d0);
  double a = -b;
  double c = y_p + z_p;
  q1_p = atan2(b, a) + atan2(sqrt(sq(a) + sq(b) - sq(c)), c);
  if ((q1_p <= (-PI / 2)) || (q1_p >= (PI / 2))) {
    q1_p = atan2(b, a) - atan2(sqrt(sq(a) + sq(b) - sq(c)), c);
  }
  q1_p *= (180 / PI);

//-------------------Compute desired joint values 4, 5 & 6 for PSM---------------------------
#ifdef DMP
  //Right MTM
  double s5, c5, s4, c4, s6, c6;
  quaternionToArray(qR, quatArray);
  quatToRotMat(quatArray, rotMat);
  arrayToMatrix(rotMat, mat);
  matrixMult(mat1, mat, result);
  s5 = result[2][1];
  c5 = sqrt(sq(result[0][1]) + sq(result[1][1]));
  q5_m = atan2(s5, c5);
  if ((q5_m <= (-PI / 2)) || (q5_m >= (PI / 2))) {
    q5_m = atan2(s5, -c5);
  }
  q5_m *= (180 / PI);

  if (q5_m != 90.0) {
    s4 = -result[0][1] / c5;
    c4 = result[1][1] / c5;
    q4_m = atan2(s4, c4);
    q4_m *= (180.0 / PI);
  } else {
    q4_m = 0.0;
  }

  if (q5_m == 90.0) {
    s6 = result[1][0];
    c6 = result[1][2];
    q6_m = atan2(s6, c6);
    q6_m *= (180.0 / PI);
  } else if (q5_m == -90.0) {
    s6 = result[1][0];
    c6 = result[1][2];
    q6_m = -atan2(s6, c6);
    q6_m *= (180.0 / PI);
  } else {
    s6 = -result[2][0] / c5;
    c6 = -result[2][2] / c5;
    q6_m = atan2(s6, c6);
    q6_m *= (180.0 / PI);
  }

  q7_m = 1.261157 + (53481730 - 1.261157) / (1 + pow((HallR / 84.42502), 8.110327));

  // Remap values to PSM
  q4_p = q6_m;  //PSM Roll
  q5_p = q4_m;  //PSM Pitch
  q6_p = -q5_m;  //PSM Yaw
  q7_p = -2 * (q7_m - 1.37);
#endif

  //-----------------------Send data over UART---------------------------------
  String buffer = "";
  send_time = millis();
  buffer = compData(q1_p, q2_p, q3_p, q4_p, q5_p, q6_p, q7_p, send_time, 2);
  //Check if enough bytes are available for writing
  int dataSize = buffer.length();
  if (Serial2.availableForWrite() >= dataSize) {
    Serial2.println(buffer);
    Serial.println(buffer);
  } else {
    Serial.println("Serial2 buffer is full. Data not sent.");
  }

#ifdef EVAL
  //SerialPrintData(4);
#endif
}

//=======================================================
//======               Functions                  =======
//=======================================================
String compData(double in_value1, double in_value2, double in_value3, double in_value4, double in_value5, double in_value6, double in_value7, unsigned long time, byte signi) {
  char buffer[81];
  String serialData;

  dtostrf(in_value1, 0, 2, buffer);
  serialData = "<";
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value2, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value3, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value4, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value5, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value6, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value7, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  ltoa(time, buffer, 10);
  serialData += buffer;
  serialData += ">";

  return serialData;
}

void euler_to_rotation_matrix(float roll, float pitch, float yaw, float R[3][3]) {
  R[0][0] = cos(yaw) * cos(pitch);
  R[0][1] = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
  R[0][2] = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
  R[1][0] = sin(yaw) * cos(pitch);
  R[1][1] = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
  R[1][2] = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
  R[2][0] = -sin(pitch);
  R[2][1] = cos(pitch) * sin(roll);
  R[2][2] = cos(pitch) * cos(roll);
}

void quatToRotMat(double quat[4], double rotMat[9]) {
  double x2 = quat[0] * quat[0];
  double y2 = quat[1] * quat[1];
  double z2 = quat[2] * quat[2];
  double xy = quat[0] * quat[1];
  double xz = quat[0] * quat[2];
  double yz = quat[1] * quat[2];
  double wx = quat[3] * quat[0];
  double wy = quat[3] * quat[1];
  double wz = quat[3] * quat[2];

  rotMat[0] = 1.0 - 2.0 * (y2 + z2);
  rotMat[1] = 2.0 * (xy - wz);
  rotMat[2] = 2.0 * (xz + wy);
  rotMat[3] = 2.0 * (xy + wz);
  rotMat[4] = 1.0 - 2.0 * (x2 + z2);
  rotMat[5] = 2.0 * (yz - wx);
  rotMat[6] = 2.0 * (xz - wy);
  rotMat[7] = 2.0 * (yz + wx);
  rotMat[8] = 1.0 - 2.0 * (x2 + y2);
}

void arrayToMatrix(double arr[9], double mat[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      mat[i][j] = arr[i * 3 + j];
    }
  }
}

void matrixMult3(double mat1[3][3], double mat2[3][3], double mat3[3][3], double result[3][3]) {
  double temp[3][3];
  matrixMult(mat1, mat2, temp);
  matrixMult(temp, mat3, result);
}

void matrixMult(double mat1[3][3], double mat2[3][3], double result[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        result[i][j] += mat1[i][k] * mat2[k][j];
      }
    }
  }
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

void quaternionToArray(Quaternion quat, double arr[4]) {
  arr[0] = quat.x;
  arr[1] = quat.y;
  arr[2] = quat.z;
  arr[3] = quat.w;
}

void updateArray(float *arr1, float *arr2) {
  for (int i = 0; i < (sizeof(arr1) / sizeof(arr1[0])); i++) {
    // Serial.println(sizeof(arr1) / sizeof(arr1[0]));
    arr1[i] = arr2[i];
  }
}

void updateArray(unsigned int *arr1, unsigned int *arr2) {
  for (int i = 0; i < (sizeof(arr1) / sizeof(arr1[0])); i++) {
    arr1[i] = arr2[i];
  }
}

void updateArray(int *arr1, int *arr2) {
  for (int i = 0; i < (sizeof(arr1) / sizeof(arr1[0])); i++) {
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

void getDT() {  // loop cycle time calculation
  now = millis();
  dt = (now - past) / 1000.0;
  past = now;
}

void Initcalib() {  // 10 time averaging  - initialization
  double sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0, sumGyY = 0, sumGyZ = 0;
  readRawData();
  for (int i = 0; i < 10; i++) {
    readRawData();
    sumAcX += AcX;
    sumAcY += AcY;
    sumAcZ += AcZ;
    sumGyX += GyX;
    sumGyY += GyY;
    sumGyZ += GyZ;
    delay(50);
  }
  averAcX = sumAcX / 10;
  averAcY = sumAcY / 10;
  averAcZ = sumAcY / 10;
  averGyX = sumGyX / 10;
  averGyY = sumGyY / 10;
  averGyZ = sumGyZ / 10;
}

void initSensor() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR_R);  // I2C address
  Wire.write(0x6B);                    // Power Management Register 107, starting communication
  Wire.write(0);                       // wakes up the MPU-6050
  Wire.endTransmission(true);
}

void readRawData() {
  Wire.beginTransmission(MPU_ADDR_R);
  Wire.write(0x3B);  // AcX address
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR_R, 14, true);  // 14 Byte after AcX address

  AcX = Wire.read() << 8 | Wire.read();  //Merging 2 Bytes by OR operator and the shift operators
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}
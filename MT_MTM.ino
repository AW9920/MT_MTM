//Define Arduino UNO CPU clock
#define F_CPU 16000000L

/*Include Libraries
  #include "Uduino.h"  // Include Uduino library at the top of the sketch
  Uduino uduino("IMU");*/

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

//create IMU Object
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 mpu(0x69); // <-- use for AD0 high
MPU6050 mpu;

// Causes a Null Operation which has no effect.
//#define NOP __asm__ __volatile__ ("nop\n\t")        //Skip one single tick. 1NOP = 1tick = 62.5ns
#define DELAY_CYCLES(n) __builtin_avr_delay_cycles(n)  //Skips n ticks. delay = n*62.5ns /n is integer
unsigned int const delay_375ns = 6;
unsigned int const delay_500ns = 8;

//=======================================================
//======             GLOBAL VARIABLES             =======
//=======================================================
// MPU control/status vars
bool dmpReady;           // set true if DMP init was successful
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// Encoder control/status vars. Unity calculates true angle
/*int bit_res = 4096, FSR = 360;  //FSR = Full-Range-Scale    bit_res = Data Bits
  float angle_res = (float)FSR / (float)bit_res;*/

//orientation/motion vars
//Quaternion array --> q = [w, x, y, z]
Quaternion qR, qL;  //current raw quaternion container; left / right IMU
Quaternion *q[2] = { &qR, &qL };
//Variables for Low-Pass Filter
Quaternion qxn1R, qxn1L;  //previous raw value; left / right IMU
Quaternion qyn1R, qyn1L;  //previous filtered value; left / right IMU
Quaternion *qxn1[2] = { &qxn1R, &qxn1L };
Quaternion *qyn1[2] = { &qyn1R, &qyn1L };
//Variables for spike detection
Quaternion qsn1R, qsn1L;  //previous safe value; left / right IMU
Quaternion qsnR, qsnL;    //current Filter output (Debugging)
Quaternion *qsn1[2] = { &qsn1R, &qsn1L };
Quaternion *qsn[2] = { &qsnR, &qsnL };  //(Debugging)
Quaternion dqR, dqL;
Quaternion dq[2] = { dqR, dqL };
int cw, cx, cy, cz;
int c[4] = { cw, cx, cy, cz };  //subsequent spike counter
float dif_R[4], dif_L[4];
float *dif[2] = { dif_R, dif_L };
//Variables for Encoder data
unsigned int Enc1R, Enc2R, Enc3R;                        //Assigne Variable to Memory
unsigned int Enc1L, Enc2L, Enc3L;                        //Assigne Variable to Memory
unsigned int *EncDataR[3] = { &Enc1R, &Enc2R, &Enc3R };  //Pointer Array Right Encoders declaration
unsigned int *EncDataL[3] = { &Enc1L, &Enc2L, &Enc3L };  //Pointer Array Left Encoders declaration
unsigned int DataPinR[3] = { DO1R, DO2R, DO3R };         //Data Pins Encoder right arm
unsigned int DataPinL[3] = { DO1L, DO2L, DO3L };         //Data Pins Encoder left arm
unsigned int CSR[3] = { CS1R, CS2R, CS3R };              //Pointer Array Chip selection Pin Right
unsigned int CSL[3] = { CS1L, CS2L, CS3L };              //Pointer Array Chip selection Pin Left
unsigned int ADDR[2] = {
  ADR,  //Right IMU index 0
  ADL   //Left IMU index 1
};
//Hall Sensor variables
int HallR;
int HallL;

//[Shoulder_P, Shoulder_Y, Elbow]
//=======================================================
//======            FUNCTION DECLARATION          =======
//=======================================================
//Function declaration //To ease debugging
void setupIMU(unsigned int AD);
void readEncoder(unsigned int *OutData, unsigned int DO, int CSn, unsigned int CLK, int i);
void readIMU(Quaternion *q, int i);
Quaternion LPFilter(Quaternion *qxn, Quaternion *qxn1, Quaternion *qyn1);
void spikeDetection(Quaternion *qxn, Quaternion *qyn1);
void updateQuat(Quaternion *q, float *q_val);
float *Quat2floatArr(Quaternion *q);
void updateArray(float *arr1, float *arr2);
void breakpoint(void);

//Other variables
unsigned long currentTime;
unsigned long samplingTime;

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
  //Serial.println(sizeof(q) / sizeof(unsigned int));
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    Serial.println(ADDR[i]);
    if (ADDR[i] == ADR) {
      Serial.println("Setup IMU right");
      digitalWrite(AD0R, LOW);   //I2C address 0x68
      digitalWrite(AD0L, HIGH);  //I2C Address 0x69
    } else if (ADDR[i] == ADL) {
      Serial.println("Setup IMU left");
      digitalWrite(AD0R, HIGH);  //I2C address 0x69
      digitalWrite(AD0L, LOW);   //I2C Address 0x68
    } else {
      return;
    }
    setupIMU(ADDR[i]);
  }
}

//=======================================================
//======              MAIN ROUTINE                =======
//=======================================================
void loop() {
  // Update current Time at begin of sensor data sampling
  currentTime = millis();

  //Get Quaternion Data
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    if (i == ADR) {
      digitalWrite(AD0R, LOW);   //I2C address 0x68
      digitalWrite(AD0L, HIGH);  //I2C Address 0x69
    } else if (i == ADL) {
      digitalWrite(AD0R, HIGH);  //I2C address 0x69
      digitalWrite(AD0L, LOW);   //I2C Address 0x68
    } else {
      return;
    }
    //Acquire Data from IMU sensor
    readIMU(q[i], i);

    //Spike Filter
    spikeDetection(q[i], qsn1[i], dif[i]);
  }

  //Get Encoder Data Right
  for (unsigned int i = 0; i < sizeof(EncDataR) / sizeof(unsigned int); i++) {
    readEncoder(EncDataR[i], DataPinR[i], CSR[i], CLKR, i);  //Hand over Memory address of EncData and overwrite values
    delayMicroseconds(1);                                    //Tcs waiting for another read in
  }

  //Get Encoder Data Left
  for (unsigned int i = 0; i < sizeof(EncDataL) / sizeof(unsigned int); i++) {
    readEncoder(EncDataL[i], DataPinL[i], CSL[i], CLKL, i);  //Hand over Memory address of EncData and overwrite values

    delayMicroseconds(1);  //Tcs waiting for another read in
  }

  //Get Hall Sensor data
  HallR = analogRead(HDOR);
  HallL = analogRead(HDOL);

  //Filtering IMU data; Terminate outbreaks
  for (unsigned int i = 0; i < sizeof(q) / sizeof(unsigned int); i++) {
    *q[i] = LPFilter(q[i], qxn1[i], qyn1[i]);
  }

#ifdef RUN
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
#endif

#ifdef EVAL
  //*Compute sampling time
  /*
    samplingTime = millis() - currentTime;
    //Output sampling Time
    Serial.print("Sampling Time:");
    Serial.println(samplingTime);
*/

  //Serial Plot
  //*IMU values of LEFT arm
  Serial.print(qR.w, 4);
  Serial.print("\t");
  Serial.print(qR.x, 4);
  Serial.print("\t");
  Serial.print(qR.y, 4);
  Serial.print("\t");
  Serial.print(qR.z, 4);
  Serial.print("\t");
  /*
  Serial.print(qsnR.w, 4);
  Serial.print("\t");
  Serial.print(qsnR.x, 4);
  Serial.print("\t");
  Serial.print(qsnR.y, 4);
  Serial.print("\t");
  Serial.print(qsnR.z, 4);
  Serial.println("\t");
*/
  /*Encoder values of RIGHT arm
    Serial.print(Enc1R);      //Shoulder Pitch
    Serial.print("\t");
    Serial.print(Enc2R);      //Elbow
    Serial.print("\t");
    Serial.print(Enc3R);      //Shoulder Yaw
    Serial.print("\t");
    Serial.print(HallR);
    Serial.print("\t");
  */
  //*IMU values of LEFT arm
  Serial.print(qL.w, 4);
  Serial.print("\t");
  Serial.print(qL.x, 4);
  Serial.print("\t");
  Serial.print(qL.y, 4);
  Serial.print("\t");
  Serial.print(qL.z, 4);
  Serial.println("\t");

  /*Encoder values of LEFT arm
    Serial.print(Enc1L);      //Shoulder Pitch
    Serial.print("\t");
    Serial.print(Enc2L);      //Elbow
    Serial.print("\t");
    Serial.print(Enc3L);      //Shoulder Yaw
    Serial.print("\t");
    Serial.println(HallL);
  */

  //Difference between raw value and last save value
  /*
  Serial.print(dqR.w, 4);
  Serial.print("\t");
  Serial.print(dqR.x, 4);
  Serial.print("\t");
  Serial.print(dqR.y, 4);
  Serial.print("\t");
  Serial.print(dqR.z, 4);
  Serial.print("\t");

  Serial.print(dqL.w, 4);
  Serial.print("\t");
  Serial.print(dqL.x, 4);
  Serial.print("\t");
  Serial.print(dqL.y, 4);
  Serial.print("\t");
  Serial.print(dqL.z, 4);
  Serial.println("\t");
*/
  /*
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 4; j++) {
      Serial.print(dif[i][j], 4);
      Serial.print("\t");
    }
  }
  Serial.println();
*/
  /*
  Serial.print(dqR.w, 4);
  Serial.print("\t");
  Serial.print(dqR.x, 4);
  Serial.print("\t");
  Serial.print(dqR.y, 4);
  Serial.print("\t");
  Serial.print(dqR.z, 4);
  Serial.print("\t");

  Serial.print(dqL.w, 4);
  Serial.print("\t");
  Serial.print(dqL.x, 4);
  Serial.print("\t");
  Serial.print(dqL.y, 4);
  Serial.print("\t");
  Serial.print(dqL.z, 4);
  Serial.println("\t");
  */

#endif
}

void updateQuat(Quaternion *q, float *q_val) {
  q->w = q_val[0];
  q->x = q_val[1];
  q->y = q_val[2];
  q->z = q_val[3];
  return;
}

float *Quat2floatArr(Quaternion *q) {
  static float Q_f[4] = {
    q->w,
    q->x,
    q->y,
    q->z
  };
  return Q_f;
}

void updateArray(float *arr1, float *arr2) {
  for (int i = 0; i < sizeof(*arr1); i++) {
    arr1[i] = arr2[i];
  }
}

void breakpoint(void) {
  Serial.println();
  Serial.print("Debugger Breakpoint!");
  while (true) {}
}
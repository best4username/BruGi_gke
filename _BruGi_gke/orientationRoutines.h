#include <limits.h>

/*************************/
/* MPU6050 Routines      */
/*************************/
/* FS_SEL | Full Scale Range   | LSB Sensitivity
 * -------+--------------------+----------------
 * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
 * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
 * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
 * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
 */
/*
 * AFS_SEL | Full Scale Range | LSB Sensitivity
 * --------+------------------+----------------
 * 0       | +/- 2g           | 8192 LSB/mg
 * 1       | +/- 4g           | 4096 LSB/mg
 * 2       | +/- 8g           | 2048 LSB/mg
 * 3       | +/- 16g          | 1024 LSB/mg
*/

//#define GRAVITY 16384.0f
#define GRAVITY 15500.0f

void initResolutionDevider()
{
  if (MPU6050_GYRO_FS == 0x00) resolutionDevider = 131.0;
  if (MPU6050_GYRO_FS == 0x01) resolutionDevider = 65.5;
  if (MPU6050_GYRO_FS == 0x02) resolutionDevider = 32.8;
  if (MPU6050_GYRO_FS == 0x03) resolutionDevider = 16.4;
}


// This functions performs an initial gyro offset calibration
// INCLUDING motion detection
// Board should be still for some seconds

#if defined(GKE)
// Use UAVX gyro and acc calibration
#define ACC_ITERATIONS 500
#define ACC_THRESH_FAIL 1000
#define ACC_THRESH_GMIN 3000

int16_t RawGyro[3];

void ReadGyroAndTemp() {
  mpu.getRotation(&RawGyro[0], &RawGyro[1], &RawGyro[2]);
  fp_mpu_temp = mpu.readRealTemperature_gke();
}

void gyroOffsetCalibration() {

  // (C) G.K. Egan 2012
  // Basic idea from MEMSIC #AN-00MX-002 Ricardo Dao 4 Nov 2002
  // gyro and acc temperature calibration using linear compensation

  // set to slow mode during calibration
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  const float RangeT = 10.0f;
  const int16_t Samples = 300; // number of samples to be used.
  const float SamplesR = 1.0f / (float) Samples;

  uint8_t a, i;
  int16_t s;
  float fp_Offset[3];
  int32_t g[2][3];
  float t[2];
  float M[3], C[3];
  float ThresholdT, TempDiff, TempDiffR;
  
  printMessage(MSG_WARNING, F("CAUTION NO MOTION TEST YET"));

  for (a = 0; a < 3; a++) {
    for (i = 0; i < 2; i++)
      g[i][a] = t[i] = 0.0f;
    fp_Offset[a] = 0.0f;
  }

  uint8_t ts = 0;
  ThresholdT = -100.0f;
  do {
    ReadGyroAndTemp();
    if (fp_mpu_temp > ThresholdT) {
      for (s = 0; s < Samples; s++) {
        delayT1(10);
        ReadGyroAndTemp();
        t[ts] += fp_mpu_temp;
        for (a = 0; a < 3; a++)
          g[ts][a] += RawGyro[a];
      }
      ts++;
      ThresholdT = fp_mpu_temp + RangeT;
      // if (ts < 2)
      //   DoBeep(1, 1);
    } else
      delayT1(100);

  } while (ts < 2);

  for (ts = 0; ts < 2; ts++) {
    for (a = 0; a < 3; a++)
      g[ts][a] *= SamplesR;
    t[ts] *= SamplesR;
  }

  TempDiff = t[1] - t[0];
  TempDiffR = 1.0f / TempDiff;

  for (a = 0; a < 3; a++) {
    M[a] = (g[1][a] - g[0][a]) * TempDiffR;
    C[a] = g[0][a];
  }

  if (abs(TempDiff) < (RangeT * 2.0f)) {

    for (a = 0; a < 3; a++) {
      config.gyrTRef = t[0];
      config.gyrM[a] = M[a]; // gradient
      config.gyrOffset[a] = C[a];
    }

    printMessage(MSG_WARNING, F("Gyro Calibration OK"));
  }
  else
    printMessage(MSG_WARNING, F("Gyro Calibration failed"));

  initMPU();
}

#else

void gyroOffsetCalibration()
{
  int i;
#define TOL 64
#define GYRO_INTERATIONS 4000
  int16_t prevGyro[3], gyro[3];
  float fp_gyroOffset[3];
  uint8_t tiltDetected = 0;
  int calibGCounter = GYRO_INTERATIONS;

  // set to slow mode during calibration
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);

  // wait 2 seconds
  delayT1(2000);

  while (calibGCounter > 0)
  {

    if (calibGCounter == GYRO_INTERATIONS)
    {
      delayT1(700);
      mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);
      for (i = 0; i < 3; i++) {
        fp_gyroOffset[i] = 0;
        prevGyro[i] = gyro[i];
      }
    }

    mpu.getRotation(&gyro[0], &gyro[1], &gyro[2]);

    for (i = 0; i < 3; i++) {
      if (abs(prevGyro[i] - gyro[i]) > TOL) {
        tiltDetected++;
        //Serial.print(F(" i="));Serial.print(i);
        //Serial.print(F(" calibGCounter="));Serial.print(calibGCounter);
        //Serial.print(F(" diff="));Serial.print(prevGyro[i] - gyro[i]);
        //Serial.print(F(" gyroi="));Serial.print(gyro[i]);
        //Serial.print(F(" prevgyroi="));Serial.println(prevGyro[i]);
        break;
      }
    }

    for (i = 0; i < 3; i++) {
      fp_gyroOffset[i] += (float)gyro[i] / GYRO_INTERATIONS;
      prevGyro[i] = gyro[i];
    }

    calibGCounter--;
    if (tiltDetected >= 1)
    {
      printMessage(MSG_WARNING, F("Gyro Calibration failed, retrying ..."));
      calibGCounter = GYRO_INTERATIONS;
      tiltDetected = 0;
    }
  }

  for (uint8_t a = 0; a < 3; a++)
    config.gyrOffset[a] = fp_gyroOffset[a];

  // restore MPU mode
  initMPU();

  printMessage(MSG_WARNING, F("Gyro Calibration OK"));

}

#endif // GKE

//***********************************************************
//  ACC calibration
//***********************************************************


//  compensate for zero point offset
//  run acc compensation at least for two directions.
//  e.g.
//      1st run: 90 deg vertical position (pitch down)
//      2nd run   0 deg horizontal position
//
#define ACC_ITERATIONS 500
#define ACC_THRESH_FAIL 1000
#define ACC_THRESH_GMIN 3000
char accCalibration() {

  int16_t devVal[3];
  int16_t minAcc[3] = {INT_MAX, INT_MAX, INT_MAX};
  int16_t maxAcc[3] = {INT_MIN, INT_MIN, INT_MIN};

  float fp_accOffset[3] = {0, 0, 0};

  // wait 0.5 seconds
  delayT1(500);

  // read acc values, determine average/min/max
  for (int i = 0; i < ACC_ITERATIONS; i++) {
    mpu.getAcceleration(&devVal[0], &devVal[1], &devVal[2]);
    for (char j = 0; j < 3; j++) {
      fp_accOffset[j] += (float)devVal[j] / ACC_ITERATIONS;
      if (devVal[j] > maxAcc[j])
        maxAcc[j] = devVal[j];

      if (devVal[j] < minAcc[j])
        minAcc[j] = devVal[j];
    }
    delayT1(2);
  }

#if 0
  for (char j = 0; j < 3; j++) {
    Serial.print(F("avg/max/min["));
    Serial.print((int)j);
    Serial.print(F("] "));
    Serial.print(fp_accOffset[j], 3);
    Serial.print(F(" / "));
    Serial.print(maxAcc[j]);
    Serial.print(F(" / "));
    Serial.print(minAcc[j]);
    Serial.println("");
  }
#endif

  // plausibility check
  for (char j = 0; j < 3; j++)
    if ((maxAcc[j] - minAcc[j]) > ACC_THRESH_FAIL)
      return -1; // failed

  // store calibration values
  for (char j = 0; j < 3; j++)
    if (abs(fp_accOffset[j]) < ACC_THRESH_GMIN)
      config.accOffset[j] = fp_accOffset[j];

  return 0;
}



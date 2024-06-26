// Library for using MPU6050 IMU with arduino IDE
#pragma once
#ifndef imuModule_h
#define imuModule_h
#endif

#include "Arduino.h"
#include <Wire.h>

#define ACC_START 0x3B
#define GYRO_START 0x43

#define CAL_LED 2

enum axes {X=0,Y,Z};

class imuModule{

  public:
  // Sensor and attitude data
  
  // Accel
  float accVector[3];
  float accAngle[3] = {0,0,0};
  float accelMag;
  
  // Gyro
  float gyroRate[3];
  float gyroRateRaw[3];
  float gyroAngle[3] = {0,0,0};

  // Hardware info
  const int addr = 0x68; // MPU6050 I2C address
  
  //int accRange = 2;
  int accRange = 8;
  
  //int gyroRange = 250;
  int gyroRange = 1000;

  // Number of samples to average for calibration
  int calSamples = 225;
  
  // Methods
  void init();
  void readGyro(bool integrate);
  void readAcc();
  void readIMU(bool integrate);
  void calibrate();
  float getDt() {return dt;};

  private:

  // Sensor biases
  float xAccBias = 0;
  float yAccBias = 0;
  float zAccBias = 0;

  float xGyroBias = 0;
  float yGyroBias = 0;
  float zGyroBias = 0;

  // Time variables
  float lastTime = 0;
  float currentTime = 0;
  float dt = 0;

};

#include "imuModule.h"


void imuModule::readGyro(bool integrate){
  
  Wire.beginTransmission(this->addr);
  Wire.write(GYRO_START); // Point to first Gyro register
  Wire.endTransmission(false);
  Wire.requestFrom(this->addr, 6, true); // Read all axes

  // Record timestep
  lastTime = currentTime;
  currentTime = millis();
  dt = (currentTime - lastTime)/1000;
  
  gyroRate[X] = (Wire.read() << 8 | Wire.read()) / (131.0/4); // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  gyroRate[Y] = (Wire.read() << 8 | Wire.read()) / (131.0/4);
  gyroRate[Z] = (Wire.read() << 8 | Wire.read()) / (131.0/4);

    // Correct for negative values
  for(int i = X; i <= Z; i++){
    if(gyroRate[i] > gyroRange)
    {
      gyroRate[i] -= 2*gyroRange;
    }
  }

  // Remove Biases
  gyroRate[X] -= xGyroBias;
  gyroRate[Y] -= yGyroBias;
  gyroRate[Z] -= zGyroBias;

  if(integrate){
    // Integrate angular rates to estimate attitude
    for(int i = X; i <= Z; i++){
      gyroAngle[i] = gyroAngle[i] + gyroRate[i]*dt;
    }
  }
  
}

void imuModule::readAcc(){
  
  Wire.beginTransmission(this->addr);
  Wire.write(ACC_START); // Point to first acceleration register (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(this->addr, 6, true); // Read accel data from 6 registers 
  
  accVector[X] = (Wire.read() << 8 | Wire.read()) / (16384.0/4); // X-axis
  accVector[Y] = (Wire.read() << 8 | Wire.read()) / (16384.0/4); // Y-axis
  accVector[Z] = (Wire.read() << 8 | Wire.read()) / (16384.0/4); // Z-axis

  // Correct for negative values
  for(int i = X; i <= Z; i++){
    if(accVector[i] > accRange)
    {
      accVector[i] -= 2*accRange;
    }
  }

  // Remove Biases
  accVector[X] -= xAccBias;
  accVector[Y] -= yAccBias;
  accVector[Z] -= zAccBias;

  // Get angles
  accAngle[X] = atan2(accVector[Y], accVector[Z]) * 180/M_PI;
  accAngle[Y] = atan2(-accVector[X], sqrt(accVector[Y]*accVector[Y] + accVector[Z]*accVector[Z])) * 180/M_PI;
  
  // Correct for attitudes greater than +/- 90 degrees (when unit is upside-down)
    /*if(accVector[Z] < 0.0)
	{
		if(accAngle[X] > 0){
			accAngle[X] = 180 + (accAngle[X] - 180);
		}
		else{
			accAngle[X] = 180 - (180-accAngle[X]);
		}
			
		if(accAngle[Y] > 0){
			accAngle[Y] = - (accAngle[Y] - 180);
		}
		else{
			accAngle[Y] = (-180 - accAngle[Y]);		
		}
	}*/
}

// Read all axes and compute angles
void imuModule::readIMU(){
  //readAcc();
  //accAngle[X] = (atan(accVector[Y] / sqrt(pow(accVector[X], 2) + pow(accVector[Z], 2))) * 180 / PI);
  //accAngle[Y] = (atan(-1 * accVector[X] / sqrt(pow(accVector[Y], 2) + pow(accVector[Z], 2))) * 180 / PI);  
}

// Initialization function
void imuModule::init(){
  Wire.beginTransmission(this->addr);       // Start communication with IMU
  Wire.write(0x6B);                         // Point to register 6B
  Wire.write(0x00);                         // Reset
  Wire.endTransmission(true); 

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(this->addr);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(this->addr);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);  
}

// Calibration function
void imuModule::calibrate(){

  //Serial.println("Starting calibration, hold still...");
  delay(750);
  
  float xAccBuffer[calSamples];
  float yAccBuffer[calSamples];
  float zAccBuffer[calSamples];

  float xGyroBuffer[calSamples];
  float yGyroBuffer[calSamples];
  float zGyroBuffer[calSamples];

  // Clear previous bias measurements
  xAccBias = 0;
  yAccBias = 0;
  zAccBias = 0;

  xGyroBias = 0;
  yGyroBias = 0;
  zGyroBias = 0;
  
  // Reset gyro integrated angles
  gyroAngle[X] = 0;
  gyroAngle[Y] = 0;
  gyroAngle[Z] = 0;
  
  // Collect n samples of data with no correction applied
  for (int i = 0; i < calSamples; i++){
    // Accelerometer
    readAcc();
    xAccBuffer[i] = accVector[X];
    yAccBuffer[i] = accVector[Y];
    zAccBuffer[i] = accVector[Z];
    
    // Hol up
    delay(5);

    // Gyro
    readGyro(false);
    xGyroBuffer[i] = gyroRate[X];
    yGyroBuffer[i] = gyroRate[Y];
    zGyroBuffer[i] = gyroRate[Z];

    // Hol up
    delay(5);
  }

  // Sum all samples
  for (int i=0; i < calSamples; i++){
    xAccBias += xAccBuffer[i];
    yAccBias += yAccBuffer[i];
    zAccBias += zAccBuffer[i]; 

    xGyroBias += xGyroBuffer[i];
    yGyroBias += yGyroBuffer[i];
    zGyroBias += zGyroBuffer[i];
  }

  // Average to find bias
  xAccBias /= calSamples;
  yAccBias /= calSamples;
  zAccBias /= calSamples;
  
  // Correct for gravity (assume sitting flat)
  zAccBias -= 1;

  xGyroBias /= calSamples;
  yGyroBias /= calSamples;
  zGyroBias /= calSamples;

  // Serial feedback (optional)
  /*Serial.print("Done! X Acc Bias: ");
  Serial.print(xAccBias);
  Serial.print(" Y Acc Bias: ");
  Serial.print(yAccBias);
  Serial.print(" Z Acc Bias: ");
  Serial.println(zAccBias);

  Serial.print("\n X Gyro Bias: ");
  Serial.print(xGyroBias);
  Serial.print(" Y Gyro Bias: ");
  Serial.print(yGyroBias);
  Serial.print(" Z Gyro Bias: ");
  Serial.println(zGyroBias);

  delay(3000);*/
      
}
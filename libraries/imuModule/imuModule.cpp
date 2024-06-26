#include "imuModule.h"

// Read gyro data and update estimate (if bool true)
void imuModule::readGyro(bool integrate){
  
  Wire.beginTransmission(this->addr);
  Wire.write(GYRO_START); // Point to first Gyro register
  Wire.endTransmission(false);
  Wire.requestFrom(this->addr, 6, true); // Read all axes
  
  // Default scalar for 250 deg/s range is 1/131. 
  // Adjusted proportionally for current range (1000 deg/s)
  gyroRate[X] = (Wire.read() << 8 | Wire.read()) / (32.75); // X-axis
  gyroRate[Y] = (Wire.read() << 8 | Wire.read()) / (32.75);	// Y-axis
  gyroRate[Z] = (Wire.read() << 8 | Wire.read()) / (32.75);	// Z-axis

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
  
  // Store raw rate before correction
  gyroRateRaw[X] = gyroRate[X];
  gyroRateRaw[Y] = gyroRate[Y];
  gyroRateRaw[Z] = gyroRate[Z];

  if (accVector[Z] < 0){
	gyroRate[X] *= -1;
	gyroRate[Y] *= -1;
  }
 
 // Record timestep
  lastTime = currentTime;
  currentTime = micros();
  dt = (currentTime - lastTime)/1000000;
  
  if(integrate){
	  
    // Integrate angular rates to estimate attitude
    for(int i = X; i <= Z; i++){
      gyroAngle[i] += gyroRate[i]*dt;
    }
	
	if(gyroAngle[Y] > 180){
	  gyroAngle[Y] -= 360;

	}
	else if(gyroAngle[Y] < -180){
	  gyroAngle[Y] += 360;
	}

	if(gyroAngle[Z] > 360){
	  gyroAngle[Z] -= 360;
	}
	else if(gyroAngle[Z] < -360){
	  gyroAngle[Z] += 360;
	}	
  }
  
}

// Read accelerometer data and compute angles 
void imuModule::readAcc(){
  
  Wire.beginTransmission(this->addr);
  Wire.write(ACC_START); // Point to first acceleration register (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(this->addr, 6, true); // Read accel data from all 6 registers 
  
  // Default scalar for 2g range is 1/16384. 
  // Adjusted proportionally for current range (8g)
  accVector[X] = (Wire.read() << 8 | Wire.read()) / (4096.0); // X-axis
  accVector[Y] = (Wire.read() << 8 | Wire.read()) / (4096.0); // Y-axis
  accVector[Z] = (Wire.read() << 8 | Wire.read()) / (4096.0); // Z-axis

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
  
  accelMag = sqrt(pow(accVector[X],2) + pow(accVector[Y],2) + pow(accVector[Z],2));

  // Get angles
  //accAngle[X] = atan2(accVector[Y], accVector[Z]) * 180/M_PI;
  //accAngle[Y] = atan2(-accVector[X], sqrt(accVector[Y]*accVector[Y] + accVector[Z]*accVector[Z])) * 180/M_PI;
  
  accAngle[X] = (atan(accVector[Y] / sqrt(pow(accVector[X], 2) + pow(accVector[Z], 2))) * 180 / PI);
  accAngle[Y] = (atan(-1 * accVector[X] / sqrt(pow(accVector[Y], 2) + pow(accVector[Z], 2))) * 180 / PI);
  
  // Adjust for attitudes greater than +/- 90 degrees (when unit is upside-down)
  // This works but creates some discontinuities in the signal when transitioning from +180 to -180
  // It also allows for constant correction of the gyro estimate
    /*if(accVector[Z] < 0.0)
	{				
		if(accAngle[Y] > 0){
			accAngle[Y] = - (accAngle[Y] - 180);
		}
		else{
			accAngle[Y] = (-180 - accAngle[Y]);		
		}
	}*/
}

// Read all axes
void imuModule::readIMU(bool integrate){
	readAcc();
	readGyro(integrate);
}

// Initialization function
void imuModule::init(){
	
  pinMode(CAL_LED, OUTPUT);
  
  Wire.beginTransmission(this->addr);       // Start communication with IMU
  Wire.write(0x6B);                         // Point to register 6B
  Wire.write(0x00);                         // Reset
  Wire.endTransmission(true); 

  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(this->addr);
  Wire.write(0x1C);                  // Point to ACCEL_CONFIG register
  Wire.write(0x10);                  // Set register to 8g full scale range
  Wire.endTransmission(true);
  
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(this->addr);
  Wire.write(0x1B);                   // Point to GYRO_CONFIG register
  Wire.write(0x10);                   // Set register to 1000deg/s full scale
  Wire.endTransmission(true);  
}

// Calibration function
void imuModule::calibrate(){

  // Signal calibration start
  digitalWrite(CAL_LED, HIGH);
  delay(120);
  digitalWrite(CAL_LED, LOW);
  
  // Give time for motion to settle
  delay(2000);
  
  // Signal calibration running
  digitalWrite(CAL_LED, HIGH);
  
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
  for(int i = 0; i <= Z; i++){
	  gyroAngle[i] = 0;
  }

  
  // Collect n samples of data with no bias correction applied
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
	
  // Calibration done
  digitalWrite(CAL_LED, LOW);
  
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
	
  delay(3000); // Delay to read data*/
      
}
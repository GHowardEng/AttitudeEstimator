// Read data from MPU6050 IMU and estimate attitude with complementary filter. 
// Includes logic to correct gyro estimate and shift weights of filter as needed.
// Grant Howard 
// 2021-06-05

// 
// X Angle - Pitch UP POSITIVE;
// Y Angle - Roll RIGHT POSITIVE;
// Z Angle - Yaw LEFT POSITIVE;

#include "imuModule.h"

// Calibration button
#define BUTTON 34

// Send serial data every 10 ms (100 Hz)
#define PRINT_PERIOD 10

// Period for accel sampling (3ms, 333 Hz)
#define ACC_PERIOD 3

// Window size for averaging accel data
#define N_ACC_WINDOW 25

// Period for gyro sampling (250us, 4kHz)
#define GYRO_PERIOD 250

// Window size for filtering final fused output
#define N_FUSE_WINDOW 10

// Keep track of timing for periodic events
unsigned long printTime = 0;
unsigned long accTime = 0;
unsigned long gyroTime = 0;
unsigned long sysTime = 0;
unsigned long resetTime = 0;

// Instantiate IMU object
imuModule imu;

void setup() {
  Wire.begin();           // Init serial bus
  Wire.setClock(1000000); // I2C clock. Run fast to speed up sampling!

  // Running serial at a high rate to minimize extra latency. 
  // Can eventually be removed if better communication/visualization implemented
  Serial.begin(500000);
  
  // Setup imu;
  imu.init();
  imu.calibrate();

  // Pin setup
  pinMode(BUTTON, INPUT);
}

// Main Loop
void loop() {

  // Setup arrays for running filtering
  int accSamp = 0;
  float xAccBuffer[N_ACC_WINDOW];
  float yAccBuffer[N_ACC_WINDOW];
  float accelAngleFiltered[2];
  float pitchAngle = 0;

  int fuseSamp = 0;
  float xFusedBuffer[N_FUSE_WINDOW];
  float yFusedBuffer[N_FUSE_WINDOW];
  float fusedFiltered[2];

  while(1){
    // Run calibration on button press
    if(!digitalRead(BUTTON)){
      imu.calibrate();
    }
  
    // Get current time
    sysTime = millis();

    ///////////////////////////////////////////
    // Sample accel 
    if(sysTime >= accTime + ACC_PERIOD){
      accTime += ACC_PERIOD;
      imu.readAcc();

      // Buffer samples
      xAccBuffer[accSamp] = imu.accAngle[X];
      yAccBuffer[accSamp] = imu.accAngle[Y];

      // Reset buffer index
      if(++accSamp >= N_ACC_WINDOW)
        accSamp = 0;

      // Clear average
      accelAngleFiltered[X] = 0;
      accelAngleFiltered[Y] = 0;

      // Apply moving average
      for (int i = 0; i < N_ACC_WINDOW; i++){
        accelAngleFiltered[X] += xAccBuffer[i];
        accelAngleFiltered[Y] += yAccBuffer[i];
      }
      
       accelAngleFiltered[X] /= N_ACC_WINDOW;
       accelAngleFiltered[Y] /= N_ACC_WINDOW;

       // Fixed-gain observer to correct estimates. Check acceleration magnitude
       if(abs(imu.accelMag) < 1.02 && abs(imu.accelMag) > 0.98){
          imu.gyroAngle[X] = imu.gyroAngle[X] + 0.025*(accelAngleFiltered[X] - imu.gyroAngle[X]);
          imu.inertialAngle[X] = imu.inertialAngle[X] + 0.025*(accelAngleFiltered[X] - imu.inertialAngle[X]);
          
          imu.gyroAngle[Y] = imu.gyroAngle[Y] + 0.025*(accelAngleFiltered[Y] - imu.gyroAngle[Y]);
       }
    }

    ////////////////////////////////////////
    // Read gyro data and run fusion at 4kHz   
    if(micros() >= gyroTime + GYRO_PERIOD){
      gyroTime += GYRO_PERIOD;
      
      imu.readGyro(true);

      // Fuse gyro and accel roll angle with comp. filter
      imu.fusedAngle[Y] = 0.99*imu.gyroAngle[Y] + 0.01*accelAngleFiltered[Y]; 

      // Blend X,Z gyros to better estimate pitch rate relative to inertial frame
      imu.inertialRate[X] = abs(cos(imu.fusedAngle[Y]*M_PI/180))*imu.gyroRate[X] + sin(imu.fusedAngle[Y]*M_PI/180)*imu.gyroRate[Z];
      imu.inertialAngle[X] += imu.inertialRate[X] * imu.getDt();
      
      // Fuse gyro and accel data with comp. filter
      if(abs(imu.fusedAngle[Y]) > 15){
        imu.fusedAngle[X] = 0.99*imu.inertialAngle[X] + 0.01*accelAngleFiltered[X];
        imu.gyroAngle[X] = imu.fusedAngle[X]; 
      }
      else{
        imu.fusedAngle[X] = 0.99*imu.gyroAngle[X] + 0.01*accelAngleFiltered[X]; 
      }
              
      // Buffer samples
      xFusedBuffer[fuseSamp] = imu.fusedAngle[X];
      yFusedBuffer[fuseSamp] = imu.fusedAngle[Y];
   
      // Reset buffer index
      if(++fuseSamp >= N_FUSE_WINDOW)
        fuseSamp = 0;
        
      // Clear averages  
      fusedFiltered[X] = 0;
      fusedFiltered[Y] = 0;

      // Apply moving average
      for (int i = 0; i < N_FUSE_WINDOW; i++){
        fusedFiltered[X] += xFusedBuffer[i];
        fusedFiltered[Y] += yFusedBuffer[i];
      }
      
       fusedFiltered[X] /= N_FUSE_WINDOW;
       fusedFiltered[Y] /= N_FUSE_WINDOW;
      
      // Z data only from gyro
      imu.fusedAngle[Z] = imu.gyroAngle[Z];
    }
  
    // Rate limited serial output to plotter
    if(sysTime >= printTime + PRINT_PERIOD){
      printTime +=PRINT_PERIOD;

      // Fused data    
      Serial.print(fusedFiltered[X]);
      Serial.print(" ");
      
      Serial.print(fusedFiltered[Y]);
      Serial.println(" ");
      
      //Serial.print(imu.fusedAngle[Z]);
      //Serial.println(" ");

      // Filtered accelerometer angles
      /*Serial.print(accelAngleFiltered[X]);
      Serial.print(" ");
      
      Serial.print(accelAngleFiltered[Y]);
      Serial.println(" ");*/

     //Serial.print(imu.accVector[Z]);
      //Serial.println(" ");*/
    }
  } 
}

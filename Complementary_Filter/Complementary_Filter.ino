// Read data from MPU6050 IMU and estimate attitude with complementary filter
// Grant Howard 
// 2021-06-05

// 
// X Angle - Pitch UP POSITIVE;
// Y Angle - Roll RIGHT POSITIVE;
// Z Angle - Yaw LEFT POSITIVE;

#include "imuModule.h"

// Calibration button
#define BUTTON 34

// Send serial data every x ms
#define PRINT_PERIOD 10

// Period for accel sampling 
#define ACC_PERIOD 3

// Window size for averaging accel
#define N_ACC_WINDOW 60

// Period for gyro sampling (1kHz)
#define GYRO_PERIOD 1

// Window size for filtering final fused output
#define N_FUSE_WINDOW 100

// Period for reseting integrators
#define RESET_PERIOD 50

// Keep track of timing for periodic events
unsigned long printTime = 0;
unsigned long accTime = 0;
unsigned long gyroTime = 0;
unsigned long sysTime = 0;
unsigned long resetTime = 0;

// Instantiate IMU object
imuModule imu;

void setup() {
  // Init serial bus
  Wire.begin();
  Wire.setClock(1000000); // I2C clock
  
  Serial.begin(500000);
  
  // Setup imu;
  imu.init();
  imu.calibrate();

  // Pin setup
  pinMode(BUTTON, INPUT);
}

// Main Loop
void loop() {
  
  int accSamp = 0;
  float xAccBuffer[N_ACC_WINDOW];
  float yAccBuffer[N_ACC_WINDOW];
  float accelAngleFiltered[2];

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
    }

    ////////////////////////////////////////
    // Read gyro data and run fusion at 1kHz   
    if(sysTime >= gyroTime + GYRO_PERIOD){
      gyroTime += GYRO_PERIOD;
      
      imu.readGyro(true);
    
      // Fuse gyro and accel data with comp. filter
      imu.fusedAngle[X] = 0.96*imu.gyroAngle[X] + 0.04*accelAngleFiltered[X];    
      imu.fusedAngle[Y] = 0.96*imu.gyroAngle[Y] + 0.04*accelAngleFiltered[Y];

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
    
    // Check for rest conditions at interval
    if(sysTime >= resetTime + RESET_PERIOD){
      resetTime += RESET_PERIOD;
     
      // Reset integrators if at rest (will correct for gyro drift)
      // ALTERNATIVE with complete Euler angles
      //if((abs(imu.gyroRate[X]) + abs(imu.gyroRate[Y])) < 20 )
      
      // This check for z accel and max attitudes is a fix until full euler angles implemented for accelerometer. (Capped at +/- 90)
      // Look at combined magnitude of roll/pitch rates and accelerometer angle (tune)
      if((abs(imu.gyroRate[X]) + abs(imu.gyroRate[Y])) < 35 && abs(imu.accAngle[X]) < 87 && abs(imu.accAngle[Y] < 87 && abs(imu.accVector[Z] > 0.15))){
        imu.gyroAngle[X] = 0.70*imu.gyroAngle[X] + 0.30*accelAngleFiltered[X];
        imu.gyroAngle[Y] = 0.70*imu.gyroAngle[Y] + 0.30*accelAngleFiltered[Y];
      }
    }
  
    // Rate limited serial output to plotter
    if(sysTime >= printTime + PRINT_PERIOD){
      printTime +=PRINT_PERIOD;

      // Fused data    
      Serial.print(fusedFiltered[X]);
      Serial.print(" ");
      
      Serial.print(fusedFiltered[Y]);
      //Serial.print(" ");  
      
      //Serial.print(imu.fusedAngle[Z]);
      Serial.println(" ");

      // Filtered accelerometer angles
      /*Serial.print(accelAngleFiltered[X]);
      Serial.print(" ");
      
      Serial.print(accelAngleFiltered[Y]);
      Serial.print(" ");

      Serial.print(imu.accVector[Z]);
      Serial.println(" ");*/
    }
  } 
}

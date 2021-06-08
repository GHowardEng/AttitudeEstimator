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

// Period for accel sampling (2ms, 500 Hz)
#define ACC_PERIOD 2

// Window size for averaging accel data
#define N_ACC_WINDOW 30

// Period for gyro sampling (250us, 4kHz)
#define GYRO_PERIOD 250

// Window size for filtering final fused output
#define N_FUSE_WINDOW 2

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

       // Fixed-gain observer to correct estimates

       if(!(abs(accelAngleFiltered[Y]) > 62 && abs(accelAngleFiltered[Y]) < 118)){
          imu.gyroAngle[X] = imu.gyroAngle[X] + 0.01*(accelAngleFiltered[X] - imu.gyroAngle[X]);
       }
       imu.gyroAngle[Y] = imu.gyroAngle[Y] + 0.01*(accelAngleFiltered[Y] - imu.gyroAngle[Y]);
    }

    ////////////////////////////////////////
    // Read gyro data and run fusion at 1kHz   
    if(micros() >= gyroTime + GYRO_PERIOD){
      gyroTime += GYRO_PERIOD;
      
      imu.readGyro(true);

      // Correct with reduced weight
      if(abs(accelAngleFiltered[Y]) > 58 && abs(accelAngleFiltered[Y]) < 122){
        imu.fusedAngle[X] = 0.995*imu.gyroAngle[X] + 0.005*accelAngleFiltered[X]; 
      }
      else{
        // Fuse gyro and accel data with comp. filter
        imu.fusedAngle[X] = 0.98*imu.gyroAngle[X] + 0.02*accelAngleFiltered[X]; 
      }

      // Fuse gyro and accel data with comp. filter
      imu.fusedAngle[Y] = 0.98*imu.gyroAngle[Y] + 0.02*accelAngleFiltered[Y];        
          
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
    /*if(sysTime >= resetTime + RESET_PERIOD){
      resetTime += RESET_PERIOD;
     
      // Correct estimation when angular rate is low (will compensate for gyro drift)

      // At any angle
      //if((abs(imu.gyroRate[X]) + abs(imu.gyroRate[Y])) < 20 ){
      
      // Within limited angles
      // Look at combined magnitude of roll/pitch rates and accelerometer angle (tune)
      if((abs(imu.gyroRate[X]) + abs(imu.gyroRate[Y])) < 30 && abs(imu.accAngle[X]) < 105 && abs(imu.accAngle[Y] < 105 && abs(imu.accVector[Z] > -0.3))){
       
        // Converge gyro pitch estimate towards acceleration angle measurement
        imu.gyroAngle[X] = 0.65*imu.gyroAngle[X] + 0.35*accelAngleFiltered[X];
        
        // Dont correct roll angle if pitch is near 90 degrees (accel angle discontinuous here)
        if(abs(imu.fusedAngle[X]) < 85){
          // Converge gyro roll estimate towards acceleration angle measurement
          imu.gyroAngle[Y] = 0.65*imu.gyroAngle[Y] + 0.35*accelAngleFiltered[Y];
        }
      }
    }*/
  
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

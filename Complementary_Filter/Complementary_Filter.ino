// Read data from MPU6050 IMU and estimate attitude using data from gyro and accelerometer.
// Includes logic to correct gyro estimate with fixed-gain observer.
// Grant Howard
// 2021-06-05
//
//
// X Angle - Pitch UP POSITIVE;
// Y Angle - Roll RIGHT POSITIVE;
// Z Angle - Yaw LEFT POSITIVE;

#include "imuModule.h"
#include "movingAverage.h"

// Calibration button
#define BUTTON 34
// Aux LED
#define LED 13

//////////////////////////////////////
// Tuning Parameters
//////////////////////////////////////

// Update Rates
// Update output every 10 ms (100 Hz)
#define OUTPUT_PERIOD 10

// Period for accel sampling (2ms, 500 Hz)
#define ACC_PERIOD 2

// Period for gyro sampling (500us, 2kHz)
#define GYRO_PERIOD 500

// Filter Windows
// Window size for averaging accel data
#define N_ACC_WINDOW 20

// Window for Z acceleration vector
#define Z_WINDOW 200

// Window size for filtering final fused output
#define N_FUSE_WINDOW 5

// Keep track of timing for periodic events
unsigned long outputTime = 0;
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
  pinMode(LED, OUTPUT);
}

// Main Loop
void loop() {

  // Instantiate filters and variables
  // Acceleration filtering
  movingAverage xAccFilter(N_ACC_WINDOW);
  movingAverage yAccFilter(N_ACC_WINDOW);
  movingAverage zAccFilter(Z_WINDOW);

  float accelAngleFiltered[2];
  float zFiltered = 0;

  // Output filter
  movingAverage xFusedFilter(N_FUSE_WINDOW);
  movingAverage yFusedFilter(N_FUSE_WINDOW);
  float fusedFiltered[2];

  // Extra stuff
  float output[3];
  int scalar = 1;
  //bool correctionApplied = false;
  bool flipped = false;

  while (1) {
    // Run calibration on button press
    if (!digitalRead(BUTTON)) {
      imu.calibrate();
    }

    // Get current time
    sysTime = millis();

    //////////////////////////////////////////////////////
    // Read accel at sample rate
    if (sysTime >= accTime + ACC_PERIOD) {
      accTime += ACC_PERIOD;
      imu.readAcc();

      // Filter acceleration data
      accelAngleFiltered[X] = xAccFilter.filter(imu.accAngle[X]);
      accelAngleFiltered[Y] = yAccFilter.filter(imu.accAngle[Y]);
      zFiltered = zAccFilter.filter(imu.accVector[Z]);

      // Fixed-gain observer to correct estimates. Check acceleration magnitude
      if (abs(imu.accelMag) < 1.05 && abs(imu.accelMag) > 0.95) {
        imu.gyroAngle[X] = imu.gyroAngle[X] + 0.012 * (accelAngleFiltered[X] - imu.gyroAngle[X]);
        imu.inertialAngle[X] = imu.inertialAngle[X] + 0.012 * (accelAngleFiltered[X] - imu.inertialAngle[X]);

        imu.gyroAngle[Y] = imu.gyroAngle[Y] + 0.012 * (accelAngleFiltered[Y] - imu.gyroAngle[Y]);
      }
    }

    //////////////////////////////////////////////////////
    // Read gyro data and run fusion at gyro sampling rate
    if (micros() >= gyroTime + GYRO_PERIOD) {
      gyroTime += GYRO_PERIOD;
      digitalWrite(LED, HIGH);

      // Read data
      imu.readGyro(true);
      
      // Invert rate if neccessary
      if (abs(imu.inertialAngle[X] > 90)) {
        scalar = -1;
      }
      else {
        scalar = 1;
      }

      // Blend X,Z gyros to better estimate pitch rate and yaw rate relative to inertial frame
      imu.inertialRate[X] = abs(cos(imu.fusedAngle[Y] * PI / 180)) * imu.gyroRate[X] + sin(imu.fusedAngle[Y] * PI / 180) * scalar * imu.gyroRate[Z];
      imu.inertialAngle[X] += imu.inertialRate[X] * imu.getDt();

      imu.inertialRate[Z] = abs(cos(imu.fusedAngle[Y] * PI / 180)) * imu.gyroRate[Z] + sin(imu.fusedAngle[Y] * PI / 180) * -imu.gyroRateRaw[X];
      imu.inertialAngle[Z] += imu.inertialRate[Z] * imu.getDt();

      if (abs(imu.fusedAngle[Y]) > 10) {
        // Used mixed angle if over roll threshold, reset X-only estimate
        imu.fusedAngle[X] = imu.inertialAngle[X];
        imu.gyroAngle[X] = imu.fusedAngle[X];
      }
      else {
        // Else use X-only estimate, reset mixed angle
        imu.fusedAngle[X] = imu.gyroAngle[X];
        imu.inertialAngle[X] = imu.gyroAngle[X];
      }

      // Take corrected gyro angle directly
      imu.fusedAngle[Y] = imu.gyroAngle[Y];

      digitalWrite(LED, LOW);
    }

    // Output at specified update rate
    if (sysTime >= outputTime + OUTPUT_PERIOD) {
      outputTime += OUTPUT_PERIOD;

      // Filter final output
      fusedFiltered[X] = xFusedFilter.filter(imu.fusedAngle[X]);
      fusedFiltered[Y] = yFusedFilter.filter(imu.fusedAngle[Y]);
      
      // Output pitch directly (already in correct range)
      output[X] = fusedFiltered[X];
      // Output roll
      output[Y] = fusedFiltered[Y];
      // Output heading
      output[Z] = imu.inertialAngle[Z];

      if (zFiltered < -0.02) {
        flipped = true;
      }
      else {
        flipped = false;
      }

      /*float corrected=0;
        //if(imu.accVector[Z] < 0.0 && abs(imu.accelMag) < 1.05 && abs(imu.accelMag) > 0.95)
        if(zFiltered < 0.0)
        {
        if(fusedFiltered[Y] > 0){
          corrected = - (fusedFiltered[Y] - 180);
        }
        else{
          corrected = (-180 - fusedFiltered[Y]);
        }
        output[Y] = output[Y] + 0.15 * (corrected - output[Y]);
        correctionApplied = true;
        }
        else{
        }*/

      // Serial output
      Serial.print(output[X]);
      Serial.print(" ");

      Serial.print(output[Y]);
      Serial.print(" ");

      //Serial.print(output[Z]);
      //Serial.print(" ");

      Serial.print(flipped);
      Serial.println(" ");
    }
  }
}

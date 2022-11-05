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
#include "filter.h"

// Calibration button
#define BUTTON 34
// Aux LED
#define LED 13

//////////////////////////////////////
// Tuning Parameters
//////////////////////////////////////

// Update Rates
// Update output every 50 ms (20 Hz)
#define OUTPUT_PERIOD 66

// Period for accel sampling (2ms, 500 Hz)
#define ACC_PERIOD 5

// Period for gyro sampling (2000us, 500 Hz)
#define GYRO_PERIOD 5000

// Filter Coefficients
// Gyro
#define FP_GYRO 0.06912

// Accel data
#define FP_ACC 0.06912

// Z acceleration vector
#define FP_Z 0.03456 

// Filtering final fused output
#define N_FUSE_WINDOW 2

// Flag to allow yaw estimation
#define ENABLE_YAW 1

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
  filter xAccFilter(FP_ACC);
  filter yAccFilter(FP_ACC);
  filter zAccFilter(FP_Z);

  float accelAngleFiltered[2];
  float zFiltered = 0;

  // Gyro
  // filter xGyroFilter(FP_GYRO);
  // filter yGyroFilter(FP_GYRO);
  //filter zGyroFilter(FP_GYRO);

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

      // Read data
      imu.readAcc();

      // Filter acceleration data
      accelAngleFiltered[X] = xAccFilter.updateFilter(imu.accAngle[X]);
      accelAngleFiltered[Y] = yAccFilter.updateFilter(imu.accAngle[Y]);
      zFiltered = zAccFilter.updateFilter(imu.accVector[Z]);

      // Fixed-gain observer to correct estimates. Check acceleration magnitude
      if (abs(imu.accelMag) < 1.05 && abs(imu.accelMag) > 0.95) {
        imu.gyroAngle[X] = imu.gyroAngle[X] + 0.05 * (accelAngleFiltered[X] - imu.gyroAngle[X]);
        imu.inertialAngle[X] = imu.inertialAngle[X] + 0.05 * (accelAngleFiltered[X] - imu.inertialAngle[X]);

        imu.gyroAngle[Y] = imu.gyroAngle[Y] + 0.05 * (accelAngleFiltered[Y] - imu.gyroAngle[Y]);
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

    //////////////////////////////////////////////////////
    // Output at specified update rate
    if (sysTime >= outputTime + OUTPUT_PERIOD) {
      outputTime += OUTPUT_PERIOD;
      
      // Output pitch
      output[X] = imu.fusedAngle[X];
      // Output roll
      output[Y] = imu.fusedAngle[Y];
      // Output heading
      output[Z] = imu.inertialAngle[Z];

      if (zFiltered < -0.02) {
        flipped = true;
      }
      else {
        flipped = false;
      }

      // Serial output
      Serial.print(output[X]);
      Serial.print(" ");

      Serial.print(output[Y]);
      Serial.print(" ");

      if(ENABLE_YAW){
        Serial.print(output[Z]);
        Serial.print(" ");
      }

    }
  }
}

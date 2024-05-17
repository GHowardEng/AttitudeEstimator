// Read data from MPU6050 IMU and estimate attitude using data from gyro and accelerometer.
// Includes logic to correct gyro estimate with fixed-gain observer.
// Grant Howard
// 2021-06-05
//
//
// X Angle - Pitch UP POSITIVE;
// Y Angle - Roll RIGHT POSITIVE;
// Z Angle - Yaw LEFT POSITIVE;

#include "estimator.h"

// Calibration button
#define BUTTON 34
// Aux LED
#define LED 13

//////////////////////////////////////
// Tuning Parameters
//////////////////////////////////////

// Update Rates
// Update output every 50 ms (20 Hz)
#define OUTPUT_PERIOD 50

// Step estimator at this period
#define STEP_PERIOD 5

// Flag to output heading estimation
#define ENABLE_YAW 1

#define OUTPUT_RATES 1

// Keep track of timing for periodic events
unsigned long outputTime = 0;
unsigned long accTime = 0;
unsigned long gyroTime = 0;
unsigned long sysTime = 0;
unsigned long resetTime = 0;

estimator est;

void setup() {
  // Running serial at a high rate to minimize extra latency.
  // Can eventually be removed if better communication/visualization implemented
  Serial.begin(500000);

  est.init();

  // Pin setup
  pinMode(BUTTON, INPUT);
  pinMode(LED, OUTPUT);
}

// Main Loop
void loop() {
  // Run calibration on button press (rework calibration)
  if (!digitalRead(BUTTON)) {
    est.calibrate();
  }

  // Get current time
  sysTime = millis();

  // Step at update rate
  if (sysTime >= accTime + STEP_PERIOD) {
    accTime += STEP_PERIOD;
    est.step();
  }

  // Output at specified rate
  if (sysTime >= outputTime + OUTPUT_PERIOD) {
    outputTime += OUTPUT_PERIOD;
    // Serial output
    if(OUTPUT_RATES){
      Serial.print(est.bodyRate[X]);
      Serial.print(" ");

      Serial.print(est.bodyRate[Y]);
      Serial.print(" ");

      Serial.print(est.bodyRate[Z]);
      Serial.print(" ");
    }
    else{
      Serial.print(est.attitude[X]);
      Serial.print(" ");

      Serial.print(est.attitude[Y]);
      Serial.print(" ");

      if (ENABLE_YAW) {
        Serial.print(est.attitude[Z]);
        Serial.print(" ");
      }
    }

    Serial.println(" ");
  }
}

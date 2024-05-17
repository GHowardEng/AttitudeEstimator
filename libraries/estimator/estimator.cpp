#include "estimator.h"

void estimator::init(){
  Wire.begin();           // Init serial bus
  Wire.setClock(1000000); // I2C clock. Run fast to speed up sampling!
  
  // Setup imu;
  imu.init();
  this->calibrate();
}

void estimator::step(){
	int scalar = 1;

	// Read data
	imu.readIMU(true);

	// Filter acceleration data
	accelAngleFiltered[X] = xAccFilter.updateFilter(imu.accAngle[X]);
	accelAngleFiltered[Y] = yAccFilter.updateFilter(imu.accAngle[Y]);
	zFiltered = zAccFilter.updateFilter(imu.accVector[Z]);
    
	// Fixed-gain observer to correct estimates. Check acceleration magnitude
	if (abs(imu.accelMag) < 1.05 && abs(imu.accelMag) > 0.95) {
		imu.gyroAngle[X] = imu.gyroAngle[X] + 0.05 * (accelAngleFiltered[X] - imu.gyroAngle[X]);
		inertialAngle[X] = inertialAngle[X] + 0.05 * (accelAngleFiltered[X] - inertialAngle[X]);

		imu.gyroAngle[Y] = imu.gyroAngle[Y] + 0.05 * (accelAngleFiltered[Y] - imu.gyroAngle[Y]);
	}
	  
	// Invert rate if neccessary
	if (abs(inertialAngle[X] > PITCH_MAX)) {
		scalar = -1;
	}
	else {
		scalar = 1;
	}

	// Blend X,Z gyros to better estimate pitch rate and yaw rate relative to inertial frame
	inertialRate[X] = abs(cos(fusedAngle[Y] * PI / 180)) * imu.gyroRate[X] + sin(fusedAngle[Y] * PI / 180) * scalar * imu.gyroRate[Z];
	inertialAngle[X] += inertialRate[X] * imu.getDt();

	inertialRate[Z] = abs(cos(fusedAngle[Y] * PI / 180)) * imu.gyroRate[Z] + sin(fusedAngle[Y] * PI / 180) * -imu.gyroRateRaw[X];
	inertialAngle[Z] += inertialRate[Z] * imu.getDt();

	inertialRate[Y] = imu.gyroRate[Y];
	  
	if (abs(fusedAngle[Y]) > 10) {
		// Used mixed angle if over roll threshold, reset X-only pitch estimate with mixed (inertial) angle
		fusedAngle[X] = inertialAngle[X];
	}
	else {
		// If we're level, use X-only (pitch) estimate, reset mixed angle
		fusedAngle[X] = imu.gyroAngle[X];
		inertialAngle[X] = imu.gyroAngle[X];
	}

	// Take corrected gyro angle directly for roll
	fusedAngle[Y] = imu.gyroAngle[Y];
	  
	bodyRate[X] = imu.gyroRateRaw[X];
	bodyRate[Y] = imu.gyroRateRaw[Y];
	bodyRate[Z] = imu.gyroRateRaw[Z];
	
	if (zFiltered < -0.1) {
		flipped = true;
	}
	else {
		flipped = false;
	}
	  
	// Limit angle ranges
	if(inertialAngle[Z] > 360){
		inertialAngle[Z] -= 360;
	}
	else if(inertialAngle[Z] < -360){
		inertialAngle[Z] += 360;
	}
	
	// Output pitch
	attitude[X] = fusedAngle[X];
    // Output roll
    attitude[Y] = fusedAngle[Y];
    // Output heading
    attitude[Z] = inertialAngle[Z];
}

void estimator::calibrate(){
	inertialAngle[X] = 0;
	inertialAngle[Y] = 0;
	inertialAngle[Z] = 0;
	
	imu.calibrate();
}
#pragma once
#ifndef estimator_h
#define estimator_h
#endif

#include "Arduino.h"
#include "imuModule.h"
#include "filter.h"

// Filter Coefficients
// Gyro
#define FP_GYRO 0.06912

// Accel data
//#define FP_ACC 0.06912
#define FP_ACC 0.35

// Z acceleration vector
#define FP_Z 0.03456

#define PITCH_MAX 90

class estimator{
	
	public:
	
	// Interfacing methods
	void init();
	void step();
	void calibrate();
	
	// Rate arrays
	float inertialRate[3] 	= {0,0,0};
	float bodyRate[3] 		= {0,0,0};
	
	// Final output attitude array
	float attitude[3] 		= {0,0,0}; 

	bool flipped 			= false;
	
	private:
	imuModule imu;
	
	// Internal angle arrays
	float inertialAngle[3] 	= {0,0,0};
	float fusedAngle[3] 	= {0,0,0};

	// Acceleration filtering
	filter xAccFilter = filter(FP_ACC);
	filter yAccFilter = filter(FP_ACC);
	filter zAccFilter = filter(FP_Z);
	
	float accelAngleFiltered[2] = {0, 0};
	float zFiltered = 0;
  
};
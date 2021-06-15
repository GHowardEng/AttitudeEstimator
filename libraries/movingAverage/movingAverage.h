#pragma once
#ifndef movingAverage_h
#define movingAverage_h
#endif

#include "Arduino.h"

#define maxWindow 200

class movingAverage{

	public:
	
	movingAverage(int n);
	float filter(float x);
	float getState(){return state;};
	int getWindow(){return window;};
	void clear();
	void set(float x);
	
	private:
	
	int window;
	int sample = 0;
	std::vector<float> buffer;
	float state;
};
#pragma once
#ifndef filter_h
#define filter_h
#endif

#include "Arduino.h"
#include <vector>

class filter{

	public:
	
	filter(float fp);
	float updateFilter(float x);
	float getState(){return state;};
	int getCoeff(){return coeff;};
	void clear();
	void set(float x);
	
	private:
	
	float coeff = 0;
	float state = 0;
};
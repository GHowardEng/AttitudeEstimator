#include "movingAverage.h"


movingAverage::movingAverage(int n){
	
	if(n > maxWindow)
		n = maxWindow;
	
	window = n;
	buffer.resize(n);
}

float movingAverage::filter(float x){
	buffer[sample] = x;
	
	if(++sample >= window)
		sample = 0;
	
	state = 0;
	for (int i = 0; i < window; i++){
		state += buffer[i];
	}
	state /= window;
	
	return state;
}

void movingAverage::clear(){
	for (int i = 0; i < window; i++){
		buffer[i] = 0;
	}
}

void movingAverage::set(float x){
		for (int i = 0; i < window; i++){
		buffer[i] = x;
	}
	state = x;
}

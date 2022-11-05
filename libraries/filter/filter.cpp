#include "filter.h"


filter::filter(float fp){
	
	if(fp > 1)
		fp = 1;
	
	coeff = fp;
}

float filter::updateFilter(float x){
	
	state = coeff * x + (1-coeff) * state; // y[n] = fp * x[n] + (1-fp) * y[n-1];
	
	return state;
}

void filter::clear(){
	state = 0;
}

void filter::set(float x){
	state = x;
}

#ifndef HATARAKI_BASICMOTION_LOWPASS_FILTER_HPP_
#define HATARAKI_BASICMOTION_LOWPASS_FILTER_HPP_

#include "MathUtils.hpp"
#include <time.h>

class LowpassFilter {
private:
	double timeConstant;
	double lastValue;
	struct timespec lastTime;
	

	
public:
	LowpassFilter(double _timeConstant) : timeConstant(_timeConstant) {
		lastValue = 0;
	}

	static double filter(double previous, double input, double RC, double dT)
	{
		double alpha = dT/(RC + dT);
		return alpha * input + (1.0 - alpha) * previous;
	}	
	
	double next(double next) {
		
		return filter(lastValue,next,timeConstant,MathUtil::timeSince(lastTime));
		clock_gettime(CLOCK_REALTIME, &lastTime);
		
	}
	
	
	
};


#endif
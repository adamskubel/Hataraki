#ifndef HATARAKI_BASICMOTION_LOWPASS_FILTER_HPP_
#define HATARAKI_BASICMOTION_LOWPASS_FILTER_HPP_

#include "MathUtils.hpp"
#include "TimeUtil.hpp"

class LowpassFilter {
private:
	double timeConstant;
	double lastValue;
	bool hasLast;
	struct timespec lastTime;
	

	
public:
	LowpassFilter(double _timeConstant) : timeConstant(_timeConstant) {
		lastValue = 0;
		hasLast = false;
	}

	static double filter(double previous, double input, double RC, double dT)
	{
		double alpha = dT/(RC + dT);
		return alpha * input + (1.0 - alpha) * previous;
	}	
	
	double next(double next) {
		
		if (hasLast)
		{
			double value = filter(lastValue,next,timeConstant,TimeUtil::timeSince(lastTime));
			lastValue = value;
			TimeUtil::setNow(lastTime);
			return value;
		}
		else
		{
			TimeUtil::setNow(lastTime);
			lastValue = next;
			hasLast = true;
			return next;
		}
	}
	
	
	
};


#endif
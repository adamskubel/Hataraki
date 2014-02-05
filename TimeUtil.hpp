#ifndef HATARAKI_BASICMOTION_TIMEUTILS_HPP_
#define HATARAKI_BASICMOTION_TIMEUTILS_HPP_


#include <time.h>

#include <string>
#include <stdexcept>
#include <iostream>

#include "MathUtils.hpp"

class TimeUtil {


public:
	static double AlarmThreshold;

	static double getTimeDelta(timespec & t0, timespec & t1);
	static void setNow(struct timespec & now);	
	static double timeSince(struct timespec & sinceTime);
	static void assertTime(timespec & start, std::string message);
};


#endif
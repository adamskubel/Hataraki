#ifndef HATARAKI_BASICMOTION_TIMEUTILS_HPP_
#define HATARAKI_BASICMOTION_TIMEUTILS_HPP_

#define NanoSecondsPerSecond 1000000000.0

#include <time.h>

#include <string>
#include <stdexcept>
#include <iostream>

#include "MathUtils.hpp"

class TimeUtil {


public:
	static double AlarmThreshold;

	static void addTime(timespec & t0, double seconds, timespec & tResult);

	static double getTimeDelta(timespec & t0, timespec & t1);
	static void setNow(struct timespec & now);	
	static double timeSince(struct timespec & sinceTime);
	static double timeUntil(struct timespec & untilTime);
	static void assertTime(timespec & start, std::string message);
};


#endif
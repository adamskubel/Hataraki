#ifndef HATARAKI_BASICMOTION_TIMEUTILS_HPP_
#define HATARAKI_BASICMOTION_TIMEUTILS_HPP_

#define NanoSecondsPerSecond 1000000000.0


#include <string>
#include <stdexcept>
#include <iostream>
#include <cmath>

#ifdef __linux__
	#include <time.h>
#else
	#include <mach/clock.h>
	#include <mach/mach.h>
#endif

class TimeUtil {


public:
	static double AlarmThreshold;
	static timespec ApplicationStart;

	static void addTime(timespec & t0, double seconds, timespec & tResult);

	static double getTimeDelta(timespec & t0, timespec & t1);
	static void setNow(timespec & now);	
	static timespec getNow();	
	static double timeSince(timespec & sinceTime);
	static double timeUntil(timespec & untilTime);
	
	static double timeBetween(timespec & tStart, timespec & tEnd);

	static void assertTime(timespec & start, std::string message);
	static void assertTime(timespec & start, std::string message, double threshold);
	
	static void sleepFor(double sleepTime);
};


#endif
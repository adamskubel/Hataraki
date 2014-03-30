#ifndef HATARAKI_BASICMOTION_TM_VOLTAGE_CONV_HPP_
#define HATARAKI_BASICMOTION_TM_VOLTAGE_CONV_HPP_

#include <time.h>

#include <list>
#include <cmath>
#include <numeric>
#include <algorithm>

#include "DRV8830.hpp"
#include "MathUtils.hpp"

class TimeMultiplexedVoltageConverter {

private:
	int maxMultiplexPeriods;	
	std::list<std::pair<double,double> > voltageHistory;
	double totalTime, lastVoltage;
	double maxVoltage;

public:
	TimeMultiplexedVoltageConverter(int maxMultiplexPeriods, double maxVoltage);
	
	void setActualVoltage(double voltage);
	double nextVoltage(double elapsedTime, double targetVoltage);

	double getAverageVoltage();

	void reset();

};


#endif
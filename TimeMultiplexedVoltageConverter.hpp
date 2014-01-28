#ifndef HATARAKI_BASICMOTION_TM_VOLTAGE_CONV_HPP_
#define HATARAKI_BASICMOTION_TM_VOLTAGE_CONV_HPP_

#include <time.h>

#include <list>
#include <cmath>
#include <numeric>  

#include "DRV8830.hpp"
#include "MathUtils.hpp"

class TimeMultiplexedVoltageConverter {

private:
	int maxMultiplexPeriods;	
	std::list<double> voltageHistory;
	int cycleIndex;

public:
	TimeMultiplexedVoltageConverter(int maxMultiplexPeriods);
	
	void setActualVoltage(double voltage);
	double nextVoltage(double targetVoltage);

	double getAverageVoltage();

	void reset();

};


#endif
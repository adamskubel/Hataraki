#include "TimeMultiplexedVoltageConverter.hpp"

TimeMultiplexedVoltageConverter::TimeMultiplexedVoltageConverter(int _maxMultiplexingPeriods)
{
	this->maxMultiplexPeriods = _maxMultiplexingPeriods;
}

void TimeMultiplexedVoltageConverter::setActualVoltage(double voltage) 
{
	voltageHistory.clear();
	voltageHistory.push_back(voltage);
}


void TimeMultiplexedVoltageConverter::reset()
{
	voltageHistory.clear();
	//cycleIndex = 0;
}


double TimeMultiplexedVoltageConverter::nextVoltage(double targetVoltage)
{
	double sign = MathUtils::sgn<double>(targetVoltage);
	double steps = std::abs(DRV8830::voltageToFractionalSteps(targetVoltage));
	
	double appliedVoltage = 0;
	double stepError = steps - std::round(steps);
	
	if (steps > 6.0 && std::abs(stepError) > 0.25) 
	{		
		double upper = DRV8830::fractionalStepsToVoltage(std::ceil(steps));
		double lower = DRV8830::fractionalStepsToVoltage(std::floor(steps));

		if (voltageHistory.size() > 0)
		{			
			double v0 = (voltageHistory.back() + upper) / 2.0;
			double v1 = (voltageHistory.back() + lower) / 2.0;
			
			appliedVoltage = (std::abs(v0-targetVoltage) < std::abs(v1-targetVoltage)) ? v0 : v1;
		}
		else
		{
			//Start with the voltage closest to target
			appliedVoltage = (stepError > 0) ? lower : upper;
		}
	}
	else
	{
		appliedVoltage = DRV8830::getNearestVoltage(targetVoltage);
	}

	voltageHistory.push_back(appliedVoltage);

	if (voltageHistory.size() > maxMultiplexPeriods)
		voltageHistory.pop_front();

	return appliedVoltage*sign;
}

double TimeMultiplexedVoltageConverter::getAverageVoltage()
{
	return std::accumulate(voltageHistory.begin(), voltageHistory.end(), 0.0)/voltageHistory.size();
}

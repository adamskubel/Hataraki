#include "TimeMultiplexedVoltageConverter.hpp"

TimeMultiplexedVoltageConverter::TimeMultiplexedVoltageConverter(int _maxMultiplexingPeriods, double _maxVoltage)
{
	this->maxMultiplexPeriods = _maxMultiplexingPeriods;
	this->maxVoltage = DRV8830::getNearestVoltage(_maxVoltage);

	if (this->maxVoltage < 0)
		throw std::runtime_error("Maximum voltage must be >= 0");
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
	double sign = sgn(targetVoltage);
	double steps = std::abs(DRV8830::voltageToFractionalSteps(targetVoltage));
	
	double appliedVoltage = 0;
	double stepError = steps - std::round(steps);

	double maxStep = std::min((double)DRV8830::MaxVoltageStep,DRV8830::getNearestVoltage(maxVoltage));
	
	const double cutoffStep = 5.0;	
	const double pulseVoltage = 0.48;
	const double offVoltage = 0.24;

	if (steps > cutoffStep && std::abs(stepError) > 0.25 && steps < maxStep) 
	{		
		double upper = DRV8830::fractionalStepsToVoltage(std::ceil(steps*sign));
		double lower = DRV8830::fractionalStepsToVoltage(std::floor(steps*sign));

		if (voltageHistory.size() > 0)
		{			
			double vSum = 0, size = voltageHistory.size();
			for (auto it = voltageHistory.begin(); it != voltageHistory.end(); it++) vSum += *it;

			double vMean_upper = (vSum + upper) / (size+1);
			double vMean_lower = (vSum + lower) / (size+1);

			//Choose whatever voltage results in the closest average voltage to the setpoint

			appliedVoltage = (std::abs(vMean_upper-targetVoltage) < std::abs(vMean_lower-targetVoltage)) ? vMean_upper : vMean_lower;
		}
		else
		{
			//Start with the voltage closest to target
			appliedVoltage = (stepError > 0) ? lower : upper;
		}
	}
	//else if (steps <= cutoffStep)	
	//{		
	//	if (voltageHistory.size() > 0 && abs(voltageHistory.back()) >= pulseVoltage)
	//		appliedVoltage = DRV8830::getNearestVoltage(targetVoltage);
	//	else
	//		appliedVoltage = pulseVoltage * sign;
	//}
	else
	{
		appliedVoltage = DRV8830::getNearestVoltage(targetVoltage);
	}

	
	if (appliedVoltage > maxVoltage) appliedVoltage = maxVoltage;
	if (appliedVoltage < -maxVoltage) appliedVoltage = -maxVoltage;
	voltageHistory.push_back(appliedVoltage);

	if (voltageHistory.size() > maxMultiplexPeriods)
		voltageHistory.pop_front();

	return appliedVoltage;
}

double TimeMultiplexedVoltageConverter::getAverageVoltage()
{
	if (voltageHistory.size() > 0)
		return std::accumulate(voltageHistory.begin(), voltageHistory.end(), 0.0)/voltageHistory.size();
	else
		return 0;
}

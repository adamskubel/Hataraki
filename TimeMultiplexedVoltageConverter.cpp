#include "TimeMultiplexedVoltageConverter.hpp"

using namespace std;

TimeMultiplexedVoltageConverter::TimeMultiplexedVoltageConverter(int _maxMultiplexingPeriods, double _maxVoltage)
{
	this->maxMultiplexPeriods = _maxMultiplexingPeriods;
	this->maxVoltage = DRV8830::getNearestVoltage(_maxVoltage);
	this->lastVoltage = 0;
	this->totalTime =0;
	
	if (this->maxVoltage < 0)
		throw std::runtime_error("Maximum voltage must be >= 0");
}

void TimeMultiplexedVoltageConverter::setActualVoltage(double voltage) 
{
	voltageHistory.clear();
	lastVoltage = voltage;
}


void TimeMultiplexedVoltageConverter::reset()
{
	voltageHistory.clear();
}


double TimeMultiplexedVoltageConverter::nextVoltage(double elapsedTime, double targetVoltage)
{
	elapsedTime = 0.05;
	
	const double ResetThreshold = 0.5;
	
	if (abs(targetVoltage - lastVoltage) > ResetThreshold)
	{
		totalTime = 0;
		voltageHistory.clear();
	}
	else
	{
		totalTime += elapsedTime;
		voltageHistory.push_back(make_pair(elapsedTime, lastVoltage));
		
		if (voltageHistory.size() > maxMultiplexPeriods)
		{
			totalTime -= voltageHistory.front().first;
			voltageHistory.pop_front();
		}
	}
	
	
	double sign = sgn(targetVoltage);
	double steps = std::abs(DRV8830::voltageToFractionalSteps(targetVoltage));
	
	double appliedVoltage = 0;
	double stepError = steps - std::round(steps);	

	double upper = DRV8830::fractionalStepsToVoltage(std::ceil(steps*sign));
	double lower = DRV8830::fractionalStepsToVoltage(std::floor(steps*sign));
	
	if (voltageHistory.size() > 0)
	{			
		double vSum = 0, size = 0;
		double tN = elapsedTime;
		for (auto it = voltageHistory.begin(); it != voltageHistory.end(); it++)
		{
			size++;
			vSum += (it->second)*(it->first/(totalTime+tN));
		}
		double vMean_upper = vSum  + upper*(tN/totalTime);
		double vMean_lower = vSum + lower*(tN/totalTime);

		//Choose whatever voltage results in the closest average voltage to the setpoint
		appliedVoltage = (std::abs(vMean_upper-targetVoltage) < std::abs(vMean_lower-targetVoltage)) ? upper : lower;
	}
	else
	{
		//Start with the voltage closest to target
		appliedVoltage = (stepError > 0) ? lower : upper;
	}
	

	//}
	//else
	//{
	//	const bool subCutoffPulsingEnabled = false;
	//	const double pulseVoltage = 0.52;
	//	const double offVoltage = 0.24;

	//	if (subCutoffPulsingEnabled)
	//	{
	//		double upper = pulseVoltage*sign;
	//		double lower = offVoltage*sign;

	//		if (voltageHistory.size() > 0)
	//		{			
	//			double vSum = 0, size = voltageHistory.size();
	//			bool pulseExists = false;
	//			for (auto it = voltageHistory.begin(); it != voltageHistory.end(); it++)
	//			{
	//				if (abs(abs(*it) - pulseVoltage) < 0.0001) pulseExists = true;
	//				vSum += *it;
	//			}

	//			if (!pulseExists)
	//			{
	//				appliedVoltage = upper;
	//			}
	//			else
	//			{
	//				double vMean_upper = (vSum + upper) / (size+1);
	//				double vMean_lower = (vSum + lower) / (size+1);

	//				//Choose whatever voltage results in the closest average voltage to the setpoint
	//				appliedVoltage = (std::abs(vMean_upper-targetVoltage) < std::abs(vMean_lower-targetVoltage)) ? upper : lower;
	//			}
	//		}
	//		else
	//		{
	//			//Start with pulse on
	//			appliedVoltage = upper;
	//		}
	//	}
	//	else
	//	{
	//		appliedVoltage = DRV8830::getNearestVoltage(targetVoltage);
	//	}
	//}

//	double appliedVoltage = DRV8830::getNearestVoltage(targetVoltage);
	if (appliedVoltage > maxVoltage) appliedVoltage = maxVoltage;
	if (appliedVoltage < -maxVoltage) appliedVoltage = -maxVoltage;
	
	lastVoltage = appliedVoltage;

	return appliedVoltage;
}

double TimeMultiplexedVoltageConverter::getAverageVoltage()
{
//	if (voltageHistory.size() > 0)
//		return std::accumulate(voltageHistory.begin(), voltageHistory.end(), 0.0)/voltageHistory.size();
//	else
//		return 0;
//	return lastVoltage;
	
	double vSum = 0;
	for (auto it = voltageHistory.begin(); it != voltageHistory.end(); it++)
	{
		vSum += it->second;
	}
	return vSum / (voltageHistory.size());
}

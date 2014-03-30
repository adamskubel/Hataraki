#include "TimeMultiplexedVoltageConverter.hpp"

using namespace std;

TimeMultiplexedVoltageConverter::TimeMultiplexedVoltageConverter(int _maxMultiplexingPeriods, double _maxVoltage)
{
	this->maxMultiplexPeriods = _maxMultiplexingPeriods;
	this->maxVoltage = DRV8830::getNearestVoltage(_maxVoltage);
	this->totalTime =0;
	
	if (this->maxVoltage < 0)
		throw std::runtime_error("Maximum voltage must be >= 0");
}

void TimeMultiplexedVoltageConverter::setActualVoltage(double voltage) 
{
	voltageHistory.clear();
	voltageHistory.push_back(make_pair(0,voltage));
}


void TimeMultiplexedVoltageConverter::reset()
{
	totalTime = 0;
	lastVoltage = 0;
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
		if (lastVoltage != 0)
		{
			totalTime += elapsedTime;
			voltageHistory.push_back(make_pair(elapsedTime, lastVoltage));
		}
		
		if (voltageHistory.size() > maxMultiplexPeriods)
		{
			totalTime -= voltageHistory.front().first;
			voltageHistory.pop_front();
		}
	}
	
	//lol
	totalTime = elapsedTime * voltageHistory.size();
	
	
	double sign = sgn(targetVoltage);
	double steps = std::abs(DRV8830::voltageToFractionalSteps(targetVoltage));
	
	double appliedVoltage = 0;
	
	vector<double> possibleVoltages = {
		DRV8830::fractionalStepsToVoltage(std::ceil(steps)+1)*sign,
		DRV8830::fractionalStepsToVoltage(std::ceil(steps))*sign,
		DRV8830::fractionalStepsToVoltage(std::floor(steps))*sign,
		DRV8830::fractionalStepsToVoltage(std::floor(steps)-1)*sign
	};
	
	if (voltageHistory.size() > 0)
	{			
		double vSum = 0;
//		double tN = elapsedTime;
		for (auto it = voltageHistory.begin(); it != voltageHistory.end(); it++)
		{
			vSum += (it->second); //*(it->first/(totalTime+tN));
		}
		vSum /= (double)(voltageHistory.size()+1);
		
		double tP = 1.0/(voltageHistory.size()+1); // (tN/(totalTime+tN));
		
		std::sort(possibleVoltages.begin(),possibleVoltages.end(),[targetVoltage,vSum,tP](double v0, double v1)
		  {
			  double v0_c = vSum + v0 * tP;
			  double v1_c = vSum + v1 * tP;
			  
			  return std::abs(v0_c-targetVoltage) < std::abs(v1_c-targetVoltage);
		  });
	}
	else
	{
		std::sort(possibleVoltages.begin(),possibleVoltages.end(),[targetVoltage](double v0, double v1)
		  {
			  return std::abs(v0-targetVoltage) < std::abs(v1-targetVoltage);
		  });
	}
	
	appliedVoltage = possibleVoltages.front();
	
	if (appliedVoltage > maxVoltage) appliedVoltage = maxVoltage;
	if (appliedVoltage < -maxVoltage) appliedVoltage = -maxVoltage;
	
	lastVoltage = appliedVoltage;

	return appliedVoltage;
}

double TimeMultiplexedVoltageConverter::getAverageVoltage()
{
	double vSum = 0;
	for (auto it = voltageHistory.begin(); it != voltageHistory.end(); it++)
	{
		vSum += it->second;
	}
	return vSum / (voltageHistory.size());
}

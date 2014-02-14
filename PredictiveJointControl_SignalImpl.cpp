#include "PredictiveJointController.hpp"

namespace ControllerConfiguration 
{	
	//Speed Control
	const double MinVelocityRValue = 0.95;
	const double BaseSpeedControlMeasureTorque = 0.01;
	const double MaxPossibleZeroVelocity = 150;
	const double MinAverageOffsetForNonZeroVelocity = 4;

	const double TorqueSMAFilterWindowSize = 10;

	const double MinimumPredictedTorque = 0.01; //N*m

	const double MinAverageVoltageForCertainMovement = 0.56;

};

using namespace ControllerConfiguration;
using namespace std;

int PredictiveJointController::getSensorAngleRegisterValue()
{
	int result = -1;
	try 
	{
		timespec start;
		MathUtil::setNow(start);
		bus->selectAddress(servoModel->sensorAddress);
		cBusSelectTime = MathUtil::timeSince(start)*1000.0;
		
		MathUtil::setNow(start);
		unsigned char buf[1] = {AS5048Registers::ANGLE};
		bus->writeToBus(buf,1);
		cSensorWriteTime = MathUtil::timeSince(start)*1000.0;
		
		
		MathUtil::setNow(start);
		unsigned char result[2] = {0,0};
		bus->readFromBus(result,2);
		
		int angle = ((int)result[0]) << 6;
		angle += (int)result[1];
		cSensorReadTime = MathUtil::timeSince(start)*1000.0;
		
		return angle;
	}
	catch (std::runtime_error & e)
	{
		emergencyHalt("Error reading sensor. ");
		cout << "Exception in getSensorAngleRegisterValue(): " <<  e.what() << endl;
	}
	return result;
}

void PredictiveJointController::setCurrentTorqueStates()
{
	double direction = MathUtils::sgn<double>(cPlanTargetVelocity);
	//Need to invert this for some reason
	cStaticModelTorque = -PoseDynamics::getInstance().computeJointTorque(jointModel->index);
	cStaticModelRotatum = (cStaticModelTorque - lStaticModelTorque)/(cTime-lTime);
		
	cPredictedTorque = cStaticModelTorque + (servoModel->frictionTorque * -direction);
	
	//Then flip back around to get motor torque. 
	cPredictedTorque = -cPredictedTorque;
	
	//Make sure torque is the same sign as motion
	if (direction > 0)
		cPredictedTorque = std::max<double>(MinimumPredictedTorque,cPredictedTorque);
	else
		cPredictedTorque = std::min<double>(-MinimumPredictedTorque,cPredictedTorque);

	if (!isTorqueEstimateValid) 
	{
		stableTorqueEstimate = cPredictedTorque;		
	}

	if (!isControlTorqueValid) 
	{
		velocityErrorIntegral = 0;
		cControlTorque = cPredictedTorque;
		isControlTorqueValid = true;
	}

	//Torque experienced by the motor, over last N frames.
	cAverageVoltage = std::accumulate(appliedVoltageHistory.begin(), appliedVoltageHistory.end(), 0.0) / appliedVoltageHistory.size();
	cMotorTorque = servoModel->getTorqueForVoltageSpeed(cAverageVoltage, cVelocity);

	
	expectedRotationalStopDirection = (int)MathUtils::sgn<double>(-cStaticModelTorque);

	//if (std::abs(cAverageVoltage) > MinAverageVoltageForCertainMovement)
	//{
	//	expectedRotationalStopDirection = MathUtils::sgn<double>(cAverageVoltage);
	//}
}


void PredictiveJointController::setCurrentState()
{
	const int HistorySize = 12;
	const int VoltageHistorySize = 12;

	timespec start;
	TimeUtil::setNow(start);

	//Push current to last
	lStaticModelTorque = cStaticModelTorque;
	lRawSensorAngle = cRawSensorAngle;
	lTime = cTime;
	
	//Carry next to current
	cDriverCommanded = false;
	
	cVoltage = nVoltage;
	cAppliedVoltage = nAppliedVoltage;
	cTargetVoltage = nTargetVoltage;
	cDriverMode = nDriverMode;
	
	//Read new values
	cNonZeroOffsetSensorAngle = getSensorAngleRegisterValue();
	cRawSensorAngle = correctAngleForDiscreteErrors(MathUtil::subtractAngles(cNonZeroOffsetSensorAngle,jointModel->sensorZeroPosition,AS5048::PI_STEPS));
	cSensorAngle = filterAngle(cRawSensorAngle); //Filter for position
	
	cTime = TimeUtil::timeSince(controllerStartTime);
	
	appliedVoltageHistory.push_back(cAppliedVoltage);
	if (appliedVoltageHistory.size() > VoltageHistorySize) appliedVoltageHistory.pop_front();

	rawSensorAngleHistory.push_back(make_pair(cTime,cSensorAngle));
	if (rawSensorAngleHistory.size() > HistorySize) rawSensorAngleHistory.pop_front();
		
	setApproximateSpeed(rawSensorAngleHistory);

	doSavitzkyGolayFiltering();

	TimeUtil::assertTime(start,jointModel->name + ".setCurrentState()");
}

double PredictiveJointController::getAverageSpeedForWindow(int windowSize)
{
	double avgSpeed = 0;
	if (!rawSensorAngleHistory.empty())
	{
		int count = 1;
		auto it=rawSensorAngleHistory.begin();
	
		auto last = (*it); it++;
		for (;it != rawSensorAngleHistory.end() && count < windowSize; it++,count++)
		{
			avgSpeed += (it->second - last.second)/(it->first - last.first);
		}
		avgSpeed /= count;
	}
	return avgSpeed;
}

void PredictiveJointController::doSavitzkyGolayFiltering()
{
	if (!config->savitzyGolayFilteringEnabled) return;

	timespec start;
	TimeUtil::setNow(start);
	
	if (rawSensorAngleHistory.size() > (config->savitzyGolayWindowSize * 2) + 2)
	{
		vector<double> constantTimeData; 

		//Assuming constant period, but data should really be resampled
		for (auto it=rawSensorAngleHistory.begin(); it != rawSensorAngleHistory.end(); it++)
			constantTimeData.push_back(it->second);

		vector<double> filtered = SGSmoothUtil::SGSmooth(constantTimeData,config->savitzyGolayWindowSize,config->savitzyGolayPolyDegree);
		cSGFilterAngle = filtered.back();
		
		filtered = SGSmoothUtil::SGDerivative(constantTimeData,config->savitzyGolayWindowSize,config->savitzyGolayPolyDegree);
		cSGFilterVelocity = filtered.back();
	}

	TimeUtil::assertTime(start,jointModel->name + ".doSavitzkyGolayFiltering()");
}

void PredictiveJointController::setApproximateSpeed(std::list<std::pair<double, double> > history)
{
	//double guess = computeSpeed(history.back().second);
	double slope, intercept, rValue;
	if (MathUtil::linearRegression(history,slope,intercept,rValue))
	{
		cVelocity = slope;
		cVelocityApproximationError = std::pow(rValue,2);

		//Very low speeds will be noisy and have a low R value
		if (cVelocityApproximationError < MinVelocityRValue && std::abs(slope) < MaxPossibleZeroVelocity)
		{
			double avgSpeed = 0;
			auto it=history.begin();
			auto last = (*it);
			it++;
			for (;it != history.end(); it++)
			{
				avgSpeed += (it->second - last.second); ///(it->first - last.first);
			}
			avgSpeed /= history.size();

			if (std::abs(avgSpeed) < MinAverageOffsetForNonZeroVelocity)
			{
				cVelocity = 0;
				cVelocityApproximationError = 1.0;
			}
		}
	}
	else
	{
		//Infinite velocity!
		cVelocityApproximationError = 0;
	}
}

//Check if angle is ~64 steps away from the previous value. If it is, then then 6th angle bit may be flippity-flooped.
//This check is only done if it is enabled for the joint.
double PredictiveJointController::correctAngleForDiscreteErrors(double rawAngle)
{
	const double MaxCorrectedOffsetFromPrevious = 7.0;
	const double MinRValueForDisplacementPrediction = 0.8;

	double predictedDisplacement = 0;
	if (cVelocityApproximationError > MinRValueForDisplacementPrediction)
	{
		predictedDisplacement = cVelocity*samplePeriod;
	}

	if (std::abs(std::abs(rawAngle - (lRawSensorAngle+predictedDisplacement)) - 64.0) < MaxCorrectedOffsetFromPrevious)
	{
		//cout << "Bit flip detected. lTime = " << cTime << endl;
		if (rawAngle < (lRawSensorAngle+predictedDisplacement))
			rawAngle += 64.0;
		else 
			rawAngle -= 64.0;
	}
	
	return rawAngle;
}


/*
 Determining speed is non-trivial due to sensor noise.
 Sensor noise can be removed with a low-pass filter, but this may not be sufficient as the frequency
 of the noise is close to the frequency of the actual velocity changes.
 
 More investigation is needed to determine the optimal filtering strategy.
 Minimizing delay is critical to obtain smooth motion.
 */
double PredictiveJointController::computeSpeed(double rawSensorAngle)
{
	filter_sma_angle_for_speed->add(rawSensorAngle);
	double filteredAngle = filter_sma_angle_for_speed->avg();
	
	double delta = filteredAngle - lFilteredAngleForSpeed;
	lFilteredAngleForSpeed = filteredAngle;
	
	if (std::abs(cTime - lTime) > 0.000001)
	{
		return delta/(cTime - lTime);
	}
	else
		return 0;
}

double PredictiveJointController::filterAngle(int rawSensorAngle)
{
	filter_sma_angle_for_position->add(rawSensorAngle);
	return filter_sma_angle_for_position->avg();
}
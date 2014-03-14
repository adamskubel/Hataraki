#include "PredictiveJointController.hpp"

namespace ControllerConfiguration 
{	
	const double MaxPossibleZeroVelocity = 150;
	const double MinAverageOffsetForNonZeroVelocity = 4;
	const double MinimumPredictedTorque = 0.0; //N*m
};

using namespace ControllerConfiguration;
using namespace std;

int PredictiveJointController::getSensorAngleRegisterValue()
{
	int result = -1;
	try 
	{
		timespec start;
		TimeUtil::setNow(start);
		bus->selectAddress(servoModel->sensorAddress);
		
		TimeUtil::setNow(start);
		unsigned char buf[1] = {AS5048Registers::ANGLE};
		bus->writeToBus(buf,1);
		cSensorWriteTime = TimeUtil::timeSince(start)*1000.0;
		
		TimeUtil::setNow(start);
		unsigned char result[2] = {0,0};
		bus->readFromBus(result,2);
		
		int angle = ((int)result[0]) << 6;
		angle += (int)result[1];
		cSensorReadTime = TimeUtil::timeSince(start)*1000.0;
		
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
	double direction = sgn(cPlanTargetVelocity);
	//Need to invert this for some reason
	cStaticModelTorque = 0;//-PoseDynamics::getInstance().computeJointTorque(jointModel->index);
	cStaticModelRotatum = (cStaticModelTorque - lStaticModelTorque)/(cTime-lTime);

	double frictionTorque = (servoModel->frictionTorque * -direction);

	cPredictedTorque = cStaticModelTorque + frictionTorque;
	
	//Then flip back around to get motor torque. 
	cPredictedTorque = -cPredictedTorque;
	
	//Make sure torque is the same sign as motion
	if (direction > 0)
		cPredictedTorque = std::max<double>(MinimumPredictedTorque,cPredictedTorque);
	else
		cPredictedTorque = std::min<double>(-MinimumPredictedTorque,cPredictedTorque);

	if (!isControlTorqueValid) 
	{
		velocityErrorIntegral = 0;
		cControlTorque = cPredictedTorque;
		isControlTorqueValid = true;
	}

	cMotorTorque = servoModel->getTorqueForVoltageSpeed(cVoltage, cVelocity);		
}


void PredictiveJointController::setCurrentState()
{
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
	double sensorAngle = getSensorAngleRegisterValue();
	cRawSensorAngle = correctAngleForDiscreteErrors(MathUtil::subtractAngles(sensorAngle,jointModel->sensorZeroPosition,AS5048::PI_STEPS));

	cSensorAngle = filterAngle(cRawSensorAngle); 

	//Using history to check lRawSensorAngle is valid
	if (jointModel->continuousRotation && rawSensorAngleHistory.size() > 1)
	{
		const double pi = AS5048::PI_STEPS;

		//eg. -175 to 179
		if (cSensorAngle > (pi/2) && lRawSensorAngle < (-pi/2))		
			cRevolutionCount--;		
		//eg. 178 to -179
		else if (cSensorAngle < (-pi/2) && lRawSensorAngle > (pi/2))
			cRevolutionCount++;

		cSensorAngle += (cRevolutionCount * AS5048::TAU_STEPS);
	}
	
	cTime = TimeUtil::timeSince(controllerStartTime);
	
	rawSensorAngleHistory.push_back(make_pair(cTime,cSensorAngle));
	if (rawSensorAngleHistory.size() > config->positionHistorySize) rawSensorAngleHistory.pop_front();
		
	doQuadraticRegression();

	TimeUtil::assertTime(start,jointModel->name + ".setCurrentState()");
}

void PredictiveJointController::doQuadraticRegression()
{
	quadraticRegressionFilter->addPoint(cTime,cSensorAngle);
		
	if (quadraticRegressionFilter->getSize() > 3)
	{
		cQuadRegAcceleration = quadraticRegressionFilter->aTerm();
		cVelocity = quadraticRegressionFilter->dy(cTime);
		cVelocityApproximationError = quadraticRegressionFilter->rSquare();

		//Very low speeds will be noisy and have a low R value
		if (cVelocityApproximationError < 0.8 && std::abs(cVelocity) < MaxPossibleZeroVelocity)
		{
			double avgSpeed = 0;
			auto it=rawSensorAngleHistory.begin();
			auto last = (*it);
			it++;
			for (;it != rawSensorAngleHistory.end(); it++)
			{
				avgSpeed += (it->second - last.second); ///(it->first - last.first);
			}
			avgSpeed /= rawSensorAngleHistory.size();

			if (std::abs(avgSpeed) < MinAverageOffsetForNonZeroVelocity)
			{
				cVelocity = 0;
				cVelocityApproximationError = 1.0;
			}
		}
	}
	else
	{
		cVelocity = 0; 
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
		predictedDisplacement = cVelocity*Configuration::SamplePeriod;
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


double PredictiveJointController::filterAngle(int rawSensorAngle)
{
	return rawSensorAngle;
	//return filter_lowpass_position->next((double)rawSensorAngle);
}
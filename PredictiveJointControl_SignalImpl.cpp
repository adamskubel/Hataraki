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
		timespec start = TimeUtil::getNow();
		
		bus[servoModel->sensorBus]->selectAddress(servoModel->sensorAddress);
		int angle = AS5048::getSensorAngleSteps(bus[servoModel->sensorBus]);
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
	if (!isControlTorqueValid)
	{
		double direction = sgn(cPlanTargetVelocity);
		//Need to invert this for some reason
		cStaticModelTorque = 0;//-PoseDynamics::getInstance().computeJointTorque(jointModel->index);
		//cStaticModelRotatum = (cStaticModelTorque - lStaticModelTorque)/(cTime-lTime);

		double frictionTorque = (servoModel->frictionTorque * -direction);

		cPredictedTorque = cStaticModelTorque + frictionTorque;
		
		//Then flip back around to get motor torque. 
		cPredictedTorque = 0;// -cPredictedTorque;
		
		//Make sure torque is the same sign as motion
//		if (direction > 0)
//			cPredictedTorque = std::max<double>(MinimumPredictedTorque,cPredictedTorque);
//		else
//			cPredictedTorque = std::min<double>(-MinimumPredictedTorque,cPredictedTorque);

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
	lSensorAngle = cSensorAngle;
	lTime = cTime;
	lTargetAngle = cTargetAngle;
	
	//Carry next to current
	cDriverCommanded = false;	
	cVoltage = nVoltage;
	cAppliedVoltage = nAppliedVoltage;
	cTargetVoltage = nTargetVoltage;
	cDriverMode = nDriverMode;
	
	cRawSensorAngle = 0;
	//Read new values
	for (int i=0;i<config->samplesPerUpdate;i++)
	{
		double sensorAngle = getSensorAngleRegisterValue();
		double adjustedAngle = correctAngleForDiscreteErrors(MathUtil::subtractAngles(sensorAngle,jointModel->sensorZeroPosition,AS5048::PI_STEPS));
		cRawSensorAngle += (adjustedAngle)/((double)config->samplesPerUpdate);
	}
		
	//Using history to check lRawSensorAngle is valid
	if (jointModel->continuousRotation && sensorAngleHistory.size() > 1)
	{
		const double pi = AS5048::PI_STEPS;

		//eg. -175 to 179
		if (cRawSensorAngle > (pi/2) && lRawSensorAngle < (-pi/2))
			cRevolutionCount--;		
		//eg. 178 to -179
		else if (cRawSensorAngle < (-pi/2) && lRawSensorAngle > (pi/2))
			cRevolutionCount++;

		cSensorAngle = filterAngle(cRawSensorAngle) + (cRevolutionCount * AS5048::TAU_STEPS);
	}
	else
	{
		cSensorAngle = filterAngle(cRawSensorAngle);
	}
	
	cTime = TimeUtil::timeSince(controllerStartTime);
	
	sensorAngleHistory.push_back(make_pair(cTime,cSensorAngle));
	if (sensorAngleHistory.size() > config->positionHistorySize) sensorAngleHistory.pop_front();
		
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
			auto it=sensorAngleHistory.begin();
			auto last = (*it);
			it++;
			for (;it != sensorAngleHistory.end(); it++)
			{
				avgSpeed += (it->second - last.second); ///(it->first - last.first);
			}
			avgSpeed /= sensorAngleHistory.size();

			if (std::abs(avgSpeed) < MinAverageOffsetForNonZeroVelocity)
			{
				cVelocity = 0;
				cVelocityApproximationError = 1.0;
			}
		}
	}
	else
	{
		cVelocityApproximationError = 0.0;
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
		predictedDisplacement = cVelocity*(cTime-lTime);
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
	//return rawSensorAngle;
	return filter_lowpass_position->next((double)rawSensorAngle);
}
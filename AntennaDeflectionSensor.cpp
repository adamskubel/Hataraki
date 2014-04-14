#include "AntennaDeflectionSensor.hpp"

using namespace std;

AntennaDeflectionSensor::AntennaDeflectionSensor(AntennaSensorConfig config, I2CBus * sensorBus) :
	config(config)
{
	sensorAngleFilter = new LowpassFilter(config.lowpassFilterTimeConstant);
	TimeUtil::setNow(startTime);

	cTime = 0;
	lTime = 0;

	cSensorAngle = 0;
	cRawSensorAngle = 0;
	lRawSensorAngle = 0;

	cVelocity = 0;
	cVelocityApproximationError = 0;
	cQuadRegAcceleration = 0;
	steadyAntennaAngle = 0;

	deflectionState = DeflectionState::Deflecting;
	
	quadraticRegressionFilter = new QuadraticRegression(10);
	
	this->sensorBus = sensorBus;
}

void AntennaDeflectionSensor::shutdown()
{

}

void AntennaDeflectionSensor::start()
{
	setCurrentState();
	cout << "Antenna sensor has angle " << AS5048::stepsToDegrees(cSensorAngle) << endl;
}

void AntennaDeflectionSensor::resetSensor()
{
	steadyAntennaAngle = cSensorAngle;
	deflectionState = DeflectionState::Waiting;
}

void AntennaDeflectionSensor::update()
{
	setCurrentState();
	processData();
}

void AntennaDeflectionSensor::processData()
{
	double deflectionAngle = MathUtil::subtractAngles(cSensorAngle,steadyAntennaAngle,AS5048::PI_STEPS);

	switch (deflectionState)
	{
	case DeflectionState::Waiting:
		if (deflectionAngle > config.upperDeflectionThreshold ||
			deflectionAngle < config.lowerDeflectionThreshold)
		{
			deflectionState = DeflectionState::Deflecting;

			for (auto it=deflectionEventWatchers.begin(); it != deflectionEventWatchers.end(); it++)
			{
				(*it)();
			}
		}
		break;
	case DeflectionState::Deflecting:
		steadyAntennaAngle = cSensorAngle;
		break;
	case DeflectionState::Recording:		
		activeEvent->record(cSensorAngle);
		break;
	}
}

void AntennaDeflectionSensor::startRecording()
{
	if (deflectionState == DeflectionState::Deflecting)
	{
		activeEvent = make_shared<DeflectionEvent>();
		activeEvent->record(cSensorAngle);
		deflectionState = DeflectionState::Recording;
	}
}

void AntennaDeflectionSensor::stopRecording()
{
	deflectionState = DeflectionState::Deflecting;
}

shared_ptr<DeflectionEvent> AntennaDeflectionSensor::collectEventData()
{
	return activeEvent;
}

void AntennaDeflectionSensor::setCurrentState()
{
	lTime = cTime;
	lRawSensorAngle = cRawSensorAngle;
	
	cTime = TimeUtil::timeSince(startTime);
	cRawSensorAngle = 0;
	//Read new values
	
	sensorBus->selectAddress(config.sensorAddress);
	for (int i=0;i<config.samplesPerUpdate;i++)
	{
		double sensorAngle = AS5048::getSensorAngleSteps(sensorBus);
		double adjustedAngle = correctDiscreteErrors(MathUtil::subtractAngles(sensorAngle,config.zeroOffset,AS5048::PI_STEPS));
		cRawSensorAngle += (adjustedAngle)/((double)config.samplesPerUpdate);
	}
		
	cSensorAngle = cRawSensorAngle; //sensorAngleFilter->next(cRawSensorAngle);
		
	sensorAngleHistory.push_back(make_pair(cTime,cSensorAngle));
	
	if (sensorAngleHistory.size() > config.positionHistorySize) 
		sensorAngleHistory.pop_front();
		
	doQuadraticRegression();

}

int AntennaDeflectionSensor::correctDiscreteErrors(int rawAngle)
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

void AntennaDeflectionSensor::doQuadraticRegression()
{
	const double MaxPossibleZeroVelocity = 150;
	const double MinAverageOffsetForNonZeroVelocity = 4;

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

void AntennaDeflectionSensor::addDeflectionEventWatcher(std::function<void()> deflectionEventWatcher)
{
	deflectionEventWatchers.push_back(deflectionEventWatcher);
}
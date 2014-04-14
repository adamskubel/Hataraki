#ifndef HATARAKI_BASICMOTION_ANTENNA_DEFLECTION_SENSOR_HPP_
#define HATARAKI_BASICMOTION_ANTENNA_DEFLECTION_SENSOR_HPP_

#include <functional>
#include <vector>
#include <memory>

#include "AS5048.hpp"
#include "MathUtils.hpp"
#include "I2CBus.hpp"
#include "IRealtimeUpdatable.hpp"
#include "LowpassFilter.hpp"
#include "QuadraticRegression.hpp"
#include "TimeUtil.hpp"
#include "Configuration.hpp"

class AntennaSensorConfig : public ConfigurationObject {

public:
	int samplesPerUpdate;
	int sensorAddress;
	int zeroOffset;
	int positionHistorySize;

	double upperDeflectionThreshold;
	double lowerDeflectionThreshold;

	double lowpassFilterTimeConstant;

	std::string sensorBusName;
	std::string antennaName;

	AntennaSensorConfig(cJSON * rawConfig) : ConfigurationObject(rawConfig)
	{
		antennaName= getString("Name");
		
		samplesPerUpdate = getInteger("SamplesPerUpdate");
		sensorAddress = getInteger("SensorAddress");
		zeroOffset = getInteger("ZeroOffset");
		positionHistorySize = getInteger("PositionHistorySize");
		
		zeroOffset = AS5048::degreesToSteps(getDouble("ZeroOffset"));

		upperDeflectionThreshold = AS5048::degreesToSteps(getDouble("UpperDeflectionThreshold"));
		lowerDeflectionThreshold = AS5048::degreesToSteps(getDouble("LowerDeflectionThreshold"));

		lowpassFilterTimeConstant = getDouble("LowpassFilterTimeConstant");

		sensorBusName = getString("SensorBus");
	}

};

struct DeflectionEvent {

	std::vector<std::pair<double,double> > deflectionData;
	timespec eventStartTime;

	DeflectionEvent()
	{

	}
	
	void record(double deflectionAngle)
	{
		deflectionData.push_back(std::make_pair(TimeUtil::timeSince(eventStartTime),deflectionAngle));
	}

};

class AntennaDeflectionSensor : public IRealtimeUpdatable {
	
	enum class DeflectionState {
		Waiting,
		Deflecting,
		Recording
	};

private:
	AntennaSensorConfig config;
	I2CBus * sensorBus;

	timespec startTime;

	double cTime;
	double lTime;

	double cSensorAngle;
	double cRawSensorAngle;
	double lRawSensorAngle;

	double cVelocity;
	double cVelocityApproximationError;
	double cQuadRegAcceleration;
	std::list<std::pair<double,double> > sensorAngleHistory;

	LowpassFilter * sensorAngleFilter;
	QuadraticRegression * quadraticRegressionFilter;

	DeflectionState deflectionState;

	std::shared_ptr<DeflectionEvent> activeEvent;

	void setCurrentState();
	int correctDiscreteErrors(int sensorAngle);
	void doQuadraticRegression();
	void processData();
	
	std::vector<std::function<void()> > deflectionEventWatchers;

	double steadyAntennaAngle;

public:
	AntennaDeflectionSensor(AntennaSensorConfig config, I2CBus * sensorBus);
	
	void resetSensor();

	void start();
	void update();
	void shutdown();

	void startRecording();
	void stopRecording();

	void addDeflectionEventWatcher(std::function<void()> deflectionEventWatcher);

	std::shared_ptr<DeflectionEvent> collectEventData();

};

#endif
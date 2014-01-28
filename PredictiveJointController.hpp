#ifndef HATARAKI_BASICMOTION_PREDICTIVE_JOINT_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_PREDICTIVE_JOINT_CONTROLLER_HPP_

#include <time.h>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <string>

#include "cJSON.h"

#include "AS5048.hpp"
#include "DRV8830.hpp"
#include "I2CBus.hpp"
#include "Configuration.hpp"
#include "ServoModel.hpp"
#include "SimpleMovingAverage.hpp"
//#include "PoseDynamics.hpp"
#include "LowpassFilter.hpp"
#include "TimeMultiplexedVoltageConverter.hpp"
#include "ServoUtil.hpp"


class MotionInterval {

public:
	double startSpeed, endSpeed, duration;

	MotionInterval(double constantSpeed, double _duration)
	{
		this->startSpeed = constantSpeed;
		this->endSpeed = constantSpeed;
		this->duration = _duration;
	}

	MotionInterval(double _startSpeed, double _endSpeed, double _duration) {
		this->startSpeed = _startSpeed;
		this->endSpeed = _endSpeed;
		this->duration = _duration;
	}

};

class JointMotionPlan {
	
public:
	double finalAngle;
	std::vector<MotionInterval*> motionIntervals;

	JointMotionPlan() 
	{
		finalAngle = 0;
	}
	
	JointMotionPlan(MotionInterval * interval, double _finalAngle)
	{
		motionIntervals.push_back(interval);
		this->finalAngle = _finalAngle;
	}

	JointMotionPlan(std::vector<MotionInterval*> _intervals, double _finalAngle) {
		this->motionIntervals = _intervals;
		this->finalAngle = _finalAngle;
	}

	double getSpeedAtTime(double planTime) {
		
		double intervalStart = 0;
		for (auto it=motionIntervals.begin();it != motionIntervals.end(); it++)
		{
			double intervalEnd = intervalStart+(*it)->duration;
			if (planTime >= intervalStart && planTime <= intervalEnd)
			{
				return (*it)->startSpeed;
			}
			intervalStart = intervalEnd;
		}
		return 0;
	}

};

struct ControllerConfig {

	ControllerConfig(cJSON * rawConfig) {
		
	}
	
};

enum JointStatus {
	New,
	Ready,
	Active,
	Error
};

enum ControlMode {
	Disabled,
	SpeedControl,
	PositionControl,
	StepControl,
	Hold
};

enum PositionControlState {

	Stabilizing,
	Approaching,
	Missed,
	Validating
};

enum SteppingState {

	Energizing,
	Braking,
	Reading
};

enum DriverMode {
	ConstantVoltage,
	TMVoltage,
	Coast,
	Brake
};

class PredictiveJointController {
	
private:
	ControllerConfig config;
	ServoModel * servoModel;
	JointModel * jointModel;
	I2CBus * bus;

	JointStatus jointStatus;

	double samplePeriod;

	TimeMultiplexedVoltageConverter * voltageConverter;
	
	std::ofstream csvLog;
	
	//Historical states	
	SimpleMovingAverage * smaFilter;
	LowpassFilter * torqueFilter;

	LowpassFilter * filter_lowpass_angle_for_speed;
	SimpleMovingAverage * filter_sma_angle_for_speed;
	
	LowpassFilter * filter_lowpass_speed;
	double lFilteredAngleForSpeed;
	

	//Motion plan
	std::shared_ptr<JointMotionPlan> motionPlan;
	timespec planStartTime;
	
	//Current state
	int cRawSensorAngle;
	int cNonZeroOffsetSensorAngle;

	double cSensorAngle;

	double cTargetAngleDistance;
	double cTargetAngle;
	double cTargetVelocity;

	double cVelocity;	
	double cTime;
	double cJointTorque;	
	double cMotorTorque;
	
	double cVoltage;
	DriverMode cDriverMode;
	double cTargetVoltage;
	double cAppliedVoltage;

	bool cDriverCommanded;

	//Next state
	double nVoltage;
	double nTargetVoltage;
	double nAppliedVoltage;
	DriverMode nDriverMode;
	
	//Long term states
	timespec startTime;
	PositionControlState positionControlState;
	ControlMode controlMode;


//	double setpointHoldAngle;


	//Stepping states
	std::vector<double> stepVoltages;
	SteppingState steppingState;
	int stepVoltageIndex;
	struct timespec readDelayStart;
	double stepStartPosition;
	double stepInitialTargetDistance;
	int stepExpectedDirection;


	//--Member functions--//
	//-------------------//
	
	double computeSpeed(int rawSensorAngle);
	
	double filterAngle(int currentAngle);	
	void setCurrentState();
	void executeStep(double voltage);
	void commandDriver(double targetVoltage, DriverMode mode);
	void performSafetyChecks();
	int getSensorAngleRegisterValue();

	void commitCommands();

	void printState();
	void logState();
	
	void init();

public:
	PredictiveJointController (cJSON * rawConfig, I2CBus * _bus) :
		config(rawConfig)
	{
		this->bus = _bus;
		
		jointModel = new JointModel(rawConfig);
		servoModel = jointModel->servoModel;
		
		jointStatus = JointStatus::New;		
		init();
	}
	
	void prepare();
	void enable();
	void disable();
	
	void emergencyHalt();

	void validateMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan);
	void executeMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan);
	void run();

	double getMaxJointVelocity();
	double getCurrentAngle();
	
	JointModel * getJointModel();
};


#endif
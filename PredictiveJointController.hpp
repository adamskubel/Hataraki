#ifndef HATARAKI_BASICMOTION_PREDICTIVE_JOINT_CONTROLLER_HPP_
#define HATARAKI_BASICMOTION_PREDICTIVE_JOINT_CONTROLLER_HPP_

#include <time.h>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <string>
#include <fstream>
#include <locale>
#include <algorithm>
#include <list>

#include "cJSON.h"

#include "AS5048.hpp"
#include "DRV8830.hpp"
#include "I2CBus.hpp"
#include "Configuration.hpp"
#include "ServoModel.hpp"
#include "SimpleMovingAverage.hpp"
#include "LowpassFilter.hpp"
#include "TimeMultiplexedVoltageConverter.hpp"
#include "ServoUtil.hpp"
#include "PoseDynamics.hpp"


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
	Error,
	Paused
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
	Stopping,
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

enum SpeedControlState {
	Measuring,
	Stable,
	Adjusting
};



class PredictiveJointController {
	
private:
	//ControllerConfig config;
	ServoModel * servoModel;
	JointModel * jointModel;
	I2CBus * bus;

	JointStatus jointStatus;

	double samplePeriod;

	TimeMultiplexedVoltageConverter * voltageConverter;
	
	std::ofstream csvLog;
	
	//Historical states	
	LowpassFilter * filter_lowpass_for_motorTorque;	
	SimpleMovingAverage * filter_sma_for_speedController_motorTorque;
	SimpleMovingAverage * filter_sma_angle_for_position;
	//LowpassFilter * filter_lowpass_angle_for_speed;
	SimpleMovingAverage * filter_sma_angle_for_speed;	
	LowpassFilter * filter_lowpass_speed;

	double lFilteredAngleForSpeed;
	double lTime;
	double lRawSensorAngle;
	
	std::list<std::pair<double,double> > rawSensorAngleHistory;
	std::list<double> appliedVoltageHistory;

	//Motion plan
	std::shared_ptr<JointMotionPlan> motionPlan;
	timespec planStartTime;
	
	//Current state
	double cTime;

	int cRawSensorAngle;
	int cNonZeroOffsetSensorAngle;
	double cSensorAngle;

	double cTargetAngleDistance;
	double cTargetAngle;
	double cTargetVelocity;
	
	double cVelocity;		
	double cVelocityApproximationError;

	//double cModelJointTorque;	
	double cPredictedTorque;
	double cMotorTorque;
	double cDisturbanceTorque;
	
	double cVoltage;
	DriverMode cDriverMode;
	double cTargetVoltage;
	double cAppliedVoltage;	
	double cAverageVoltage;

	bool cDriverCommanded;
	int cDriverCommand;

	double cSensorWriteTime;
	double cSensorReadTime;
	double cDriverWriteTime;
	
	//Next state
	double nVoltage;
	double nTargetVoltage;
	double nAppliedVoltage;
	DriverMode nDriverMode;
	
	//Long term states
	timespec controllerStartTime;
	PositionControlState positionControlState;
	SpeedControlState speedControlState;
	ControlMode controlMode;

	//Speed control states
	double speedControlMeasureVoltage;
	double speedControlStableTorque;
	timespec speedControlMeasureStart;

	//Postion control states
	double setpointHoldAngle;
	//timespec stopTime;

	//Stepping states
	std::vector<double> stepVoltages;
	SteppingState steppingState;
	int stepVoltageIndex;
	struct timespec readDelayStart;
	double stepStartPosition;
	double stepInitialTargetDistance;
	int stepExpectedDirection;
	double stepVoltageIntegral;


	//--Member functions--//
	//-------------------//
	
	double computeSpeed(double rawSensorAngle);
	double filterAngle(int currentAngle);
	void setApproximateSpeed(std::list<std::pair<double, double> > history);
	
	double correctAngleForDiscreteErrors(double rawAngle);

	void doSpeedControl();
	void doPositionControl();
	void doStepControl();
	
	void setCurrentState();
	void executeStep(double voltage, int energizeLength);
	void commandDriver(double targetVoltage, DriverMode mode);
	void performSafetyChecks();
	int getSensorAngleRegisterValue();

	void commitCommands();

	void printState();
	void logState();
	
	void init();

	double getMaxVoltageSteps();

public:
	PredictiveJointController (JointModel * _jointModel, I2CBus * _bus, double _samplePeriod) 
	{
		this->bus = _bus;
		this->samplePeriod = _samplePeriod;
		
		jointModel = _jointModel;
		servoModel = &(jointModel->servoModel);
		
		jointStatus = JointStatus::New;		
		init();
	}
	
	void prepare();
	void enable();
	void disable();
	void pause();
	void resume();
	
	void emergencyHalt(std::string reason);

	void validateMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan);
	void executeMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan);
	void run();

	double getMaxJointVelocity();
	double getCurrentAngle();
	
	JointModel * getJointModel();
};


#endif
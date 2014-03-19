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
#include "TimeUtil.hpp"
#include "AsyncLogger.hpp"
#include "MotionPlan.hpp"
#include "QuadraticRegression.hpp"


enum JointStatus {
	New,
	Ready,
	Active,
	Error,
	Paused
};

enum StaticControlMode {
	Holding,
	Stepping
};

enum DynamicControlMode {
	Starting,
	Travelling,
	Approaching,
	Stopping
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
	Adjusting,
	Stalled
};


class PredictiveJointController {
	
private:
	ServoModel * servoModel;
	JointModel * jointModel;
	ControllerConfig * config;
	I2CBus * bus;

	JointStatus jointStatus;
	bool haltRequested;
	
	TimeMultiplexedVoltageConverter * voltageConverter;
	
	std::string logfileName;
	
	//Filters
	LowpassFilter * filter_lowpass_position;
	QuadraticRegression * quadraticRegressionFilter;

	//Historical data
	std::list<std::pair<double,double> > rawSensorAngleHistory;

	int cRevolutionCount;

	//Previous state
	double lTime;
	double lRawSensorAngle;	
	double lStaticModelTorque;
	double lVelocityError;
		
	//Motion plan
	std::shared_ptr<MotionPlan> motionPlan;
	timespec enableTime;
	bool motionPlanComplete;
	
	//Current state
	double cTime;

//	struct SensorData {
		
		int cRawSensorAngle;
		double cSensorAngle;
		
		double cVelocity;
		double cVelocityApproximationError;

		double cQuadRegVelocity;
		double cQuadRegAcceleration;
		double cQuadRegErrorValue;
//	};
		
	double cTargetAngleDistance;
	double cTargetAngle;
	double cTargetVelocity;
	double cPlanTargetVelocity;
	double cTargetAcceleration;
	
	DynamicControlMode dynamicControlMode;
	bool dynamicControl;
	StaticControlMode staticControlMode;
	double lDynamicPositionError;
	

	// struct PredictionResults {
	double cStaticModelTorque;
	double cStaticModelRotatum;
	double cPredictedTorque;
	double cMotorTorque;
	double cControlTorque;
	double cDynamicTorque;
	//};

	
	double cVoltage;
	DriverMode cDriverMode;
	double cTargetVoltage;
	double cAppliedVoltage;	

	bool cDriverCommanded;
	int cDriverCommand;

	//This is just logging stuff, completely useless otherwise
	double cSensorWriteTime;
	double cSensorReadTime;
	double cDriverWriteTime;
	double cRunTime;
	
	//Next state
	double nVoltage;
	double nTargetVoltage;
	double nAppliedVoltage;
	DriverMode nDriverMode;
	
	//------------- Long term states --------------------
	timespec controllerStartTime;	
	SpeedControlState speedControlState;

	//Torque estimation
	bool isControlTorqueValid;

	//Speed control states
	double speedControlMeasureVoltage;
	timespec speedControlMeasureStart;
	double velocityErrorIntegral;
	double speedControlIntegralGain;
	double speedControlProportionalGain;

	//Stepping states
	//struct StepControlData {
		std::vector<double> stepVoltages;
		SteppingState steppingState;
		int stepVoltageIndex;
		struct timespec readDelayStart;
		double stepStartPosition;
		double stepInitialTargetDistance;
		int stepExpectedDirection;
		double stepVoltageIntegral;
	//};
		

	//--Member functions--//
	//-------------------//
	double estimateTimeToPosition(double position);
	double filterAngle(int currentAngle);
	int getSensorAngleRegisterValue();
	double correctAngleForDiscreteErrors(double rawAngle);
	void doQuadraticRegression();

	void performSafetyChecks();

	void doPositionHoldControl();
	void doSpeedControl();
	bool doStepControl();

	void setCurrentTorqueStates();
	void setCurrentState();
	void setTargetState();
	
	void doDynamicControl();
	void doStaticControl();
	

	void executeStep(double voltage, int energizeLength, int coastStepCount);
	void executeStep(std::vector<double> & voltagePattern);
	
	void commandDriver(double targetVoltage, DriverMode mode);
	void commitCommands();

	void logState();
	void writeLogHeader();
	
	void init();

	double getMaxVoltageSteps();
	double getStableTorqueEstimate();
	double getAverageSpeedForWindow(int windowSize);

public:
	PredictiveJointController (JointModel * _jointModel, I2CBus * _bus) 
	{
		this->bus = _bus;
		
		jointModel	= _jointModel;
		servoModel	= &(jointModel->servoModel);
		config		= &(servoModel->controllerConfig);
		
		jointStatus = JointStatus::New;		
		init();
	}
	
	void prepare();
	void enable();
	void disable();
	void pause();
	void resume();
	
	void emergencyHalt(std::string reason);
	
	void validateMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan);
	void executeMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan);
	void joinMotionPlan(std::shared_ptr<MotionPlan> newMotionPlan);

	void run();

	double getMaxVelocity();
	double getCurrentAngle();
	double getCurrentVelocity();
	double getMaxAcceleration();
		
	JointModel * getJointModel();	

	std::string getJointStatusText();
	JointStatus getJointStatus();


};


#endif
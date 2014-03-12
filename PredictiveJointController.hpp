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
#include "SavitzkyGolaySmooth.hpp"
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
	Adjusting
};


//enum SetpointApproachState {
//	Stabilizing,
//	Approaching,
//	Stopping,
//	Missed,
//	Validating
//};

class PredictiveJointController {
	
private:
	//ControllerConfig config;
	ServoModel * servoModel;
	JointModel * jointModel;
	ControllerConfig * config;
	I2CBus * bus;

	JointStatus jointStatus;
	bool haltRequested;

	double samplePeriod;

	TimeMultiplexedVoltageConverter * voltageConverter;
	
	std::string logfileName;
	
	//Filters
	LowpassFilter * filter_lowpass_for_motorTorque;	
	SimpleMovingAverage * filter_sma_for_speedController_motorTorque;
	LowpassFilter * filter_lowpass_position;
	SimpleMovingAverage * filter_sma_angle_for_speed;
	LowpassFilter * filter_lowpass_speed;
	QuadraticRegression * quadraticRegressionFilter;

	//Historical data
	std::list<std::pair<double,double> > rawSensorAngleHistory;
	std::list<double> appliedVoltageHistory;

	int cRevolutionCount;

	//Previous state
	double lFilteredAngleForSpeed;
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
		int cNonZeroOffsetSensorAngle;
		double cSensorAngle;
		
		double cVelocity;
		double cVelocityApproximationError;

		double cQuadRegVelocity;
		double cQuadRegAcceleration;
		double cQuadRegErrorValue;
		
		double cSGFilterAngle;
		double cSGFilterVelocity;
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
	double cAverageVoltage;

	bool cDriverCommanded;
	int cDriverCommand;

	//This is just logging stuff, completely useless otherwise
	double cSensorWriteTime;
	double cSensorReadTime;
	double cDriverWriteTime;
	double cBusSelectTime;
	
	//Next state
	double nVoltage;
	double nTargetVoltage;
	double nAppliedVoltage;
	DriverMode nDriverMode;
	
	//------------- Long term states --------------------
	timespec controllerStartTime;	
	SpeedControlState speedControlState;

	//Torque estimation
	double stableTorqueEstimate;
	bool isTorqueEstimateValid;
	bool isControlTorqueValid;
	
	int expectedRotationalStopDirection;

	//Speed control states
	double speedControlMeasureVoltage;
	timespec speedControlMeasureStart;
	double velocityErrorIntegral;
	double speedControlIntegralGain;
	double speedControlProportionalGain;
	int startModeIndex;

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

	//User test variables
	bool flipRequestedByUser;
	int requestedFlipDirection;
	double requestedFlipVoltage;
	std::vector<double> requestedVoltagePattern;
	bool patternRequestedByUser;
	std::function<void()> externalController;

	volatile bool readyForCommand;

	//--Member functions--//
	//-------------------//
	double estimateTimeToPosition(double position);
	double computeSpeed(double rawSensorAngle);
	double filterAngle(int currentAngle);
	void setApproximateSpeed(std::list<std::pair<double, double> > history);
	int getSensorAngleRegisterValue();
	double correctAngleForDiscreteErrors(double rawAngle);
	void doSavitzkyGolayFiltering();
	void doQuadraticRegression();

	void performSafetyChecks();

	void doPositionHoldControl();
	void doSpeedControl();
	bool doStepControl(double targetAngle);
	void runExternalController();

	bool handleUserRequests();

	void setCurrentTorqueStates();
	void setCurrentState();
	void setTargetState();
	
	void doDynamicControl();
	void doStaticControl();
	

	void executeStep(double voltage, int energizeLength, int coastStepCount);
	void executeStep(std::vector<double> & voltagePattern);
	
	void commandDriver(double targetVoltage, DriverMode mode);
	void commitCommands();

	void printState();
	void logState();
	void writeLogHeader();
	
	void init();

	double getMaxVoltageSteps();
	double getStableTorqueEstimate();
	double getAverageSpeedForWindow(int windowSize);

public:
	PredictiveJointController (JointModel * _jointModel, I2CBus * _bus, double _samplePeriod) 
	{
		this->bus = _bus;
		this->samplePeriod = _samplePeriod;
		
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

	void requestFlip(int direction, double voltage);
	void requestPattern(std::vector<double> voltagePattern);
	bool jointReadyForCommand();

	void validateMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan);
	void executeMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan);
	void run();

	double getMaxVelocity();
	double getCurrentAngle();
	double getMaxAcceleration();
		
	JointModel * getJointModel();	

	std::string getJointStatusText();
	JointStatus getJointStatus();


};


#endif
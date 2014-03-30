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
#include <map>

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

	struct DataFrame {

		double Time, Angle, Velocity, TargetAngle, TargetVelocity, PlanVelocity;
		double TargetVoltage, ActualVoltage;
		double ControlTorque, ExpectedVelocity;

		DataFrame(double Time, double Angle, double Velocity, double TargetAngle, double TargetVelocity, double PlanVelocity, double TargetVoltage, double ActualVoltage, double ControlTorque, double ExpectedVelocity)
		{
			this->Time = Time;
			this->Angle = Angle;
			this->Velocity = Velocity;
			this->TargetAngle = TargetAngle;
			this->TargetVelocity = TargetVelocity;
			this->PlanVelocity = PlanVelocity;
			this->TargetVoltage = TargetVoltage;
			this->ActualVoltage = ActualVoltage;
			this->ControlTorque = ControlTorque;
			this->ExpectedVelocity = ExpectedVelocity;
		}
	};



private:
	ServoModel * servoModel;
	JointModel * jointModel;
	ControllerConfig * config;
	std::map<std::string,I2CBus*> bus;

	JointStatus jointStatus;
	bool haltRequested;
	
	TimeMultiplexedVoltageConverter * voltageConverter;
	
	std::string logfileName;
	
	//Filters
	LowpassFilter * filter_lowpass_position;
	QuadraticRegression * quadraticRegressionFilter;

	//Historical data
	std::list<DataFrame> dataHistory;
	std::list<std::pair<double,double> > rawSensorAngleHistory;

	int cRevolutionCount;

	//Previous state
	double lTime;
	int lRawSensorAngle;
	double lSensorAngle;	
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
	double cExpectedVelocity;
	
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
	PredictiveJointController (JointModel * _jointModel, std::map<std::string,I2CBus*> busMap) 
	{
		this->bus = busMap;
		
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
	virtual double getCurrentAngle();
	double getCurrentVelocity();
	double getMaxAcceleration();
	double getAngleSetpoint();
	bool isDynamicMode();

	void writeHistoryToLog();
		
	JointModel * getJointModel();	

	std::string getJointStatusText();
	JointStatus getJointStatus();


};


#endif
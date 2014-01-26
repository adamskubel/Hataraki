#ifndef HATARAKI_BASICMOTION_JOINTLOOP
#define HATARAKI_BASICMOTION_JOINTLOOP

#include <time.h>
#include <iostream>
#include <string>
#include <stdexcept> 
#include <fstream>

#include "cJSON.h"

#include "AS5048.hpp"
#include "DRV8830.hpp"
#include "I2CBus.hpp"
#include "Configuration.hpp"
#include "MathUtils.hpp"
#include "SpeedController.hpp"
#include <functional>

namespace JointState {
	const static int New = 0;
	const static int Waiting = 1;
	const static int Active  = 2;
	const static int Error = 3;
	const static int EmergencyHalt = 4;
	const static int RunningTask = 5;
}

class JointLoop {

private:
	double lastInfoTime, samplePeriod, settlingStartTime, minSettleTime;
	
	double errorSum,errorSumMaxMagnitude;
	
	int maxSettleError;

	int minAngle, 
		maxAngle, 
		zeroPosition,
		targetAngleSteps, 
		lastAngle;

	
	double kP,kI,kD;
	double motorConstant;
	double targetJointVelocity;

	double lastVoltageError;

	int driverAddress, sensorAddress;
	
	int state;
	bool logWhileWaiting;

	std::string jointName;

	I2CBus * bus;

	std::ofstream csvLog, taskCsvLog;

	bool hasSettled, isSettling;

	std::function<bool(int, double)> runningTask;
	int runningTaskIndex;
		
	SpeedController * speedController;
	
	bool canTravel, isTravelling;

	//Private members
	bool checkAngleRange(int angle);
	void initJoint();
	int transformAngle(int angle);
	void checkSettling(int error, double voltageError);
	double getMinTravelVoltageError();
	void configurePositionController(cJSON * controllerConfig);

	struct timespec startTime, lastRunTime;

	double timeSince(struct timespec & sinceTime);


public:
	JointLoop(I2CBus * bus, cJSON * jointItem, double samplePeriod);

	void setTargetAngle(double angle);
	void setAngleRange(double minAngle, double maxAngle);

	double getMinAngle();
	double getMaxAngle();

	void setControllerCoefficients(double Kp, double Ki, double Kd);

	double getCurrentAngle();

	double getMaxJointVelocity(); //rad/s

	void setTargetJointVelocity(double targetVelocity); //rad/s

	bool checkCommandValid(double targetAngle);

	void requestEmergencyHalt();
	void pauseJoint();
	void executeSpike(double voltage,int spikeLength, int zeroLength, int repeatCount);

	void run();
	void shutdown();

	void printStatus();

	bool isSettled();

	std::string getJointName();

};

#endif
#include "PredictiveJointController.hpp"



using namespace std;

void PredictiveJointController::init()
{
	const double TorqueFilterTimeConstant = 0.05;
	const double SpeedMovingAverageWindowSize = 4;
	const double SpeedFilterTimeConstant = 0.05;
	const double TorqueSMAFilterWindowSize = 10;
	
	filter_lowpass_for_motorTorque =	new LowpassFilter(TorqueFilterTimeConstant);
	filter_lowpass_position	=			new LowpassFilter(config->get("SensorFilters.LowpassPositionFilter.TimeConstant"));
	filter_sma_angle_for_speed =		new SimpleMovingAverage(SpeedMovingAverageWindowSize);
	filter_lowpass_speed =				new LowpassFilter(SpeedFilterTimeConstant);
	filter_sma_for_speedController_motorTorque =		new SimpleMovingAverage(TorqueSMAFilterWindowSize);

	voltageConverter = new TimeMultiplexedVoltageConverter(2,servoModel->maxDriverVoltage);

	quadraticRegressionFilter = new QuadraticRegression((int)config->get("SensorFilters.VelocityApproximation.WindowSize"));

	steppingState = SteppingState::Braking;
	speedControlState = SpeedControlState::Measuring;		
	dynamicControlMode = DynamicControlMode::Travelling;
	staticControlMode = StaticControlMode::Holding;
	dynamicControl = false;
	jointStatus = JointStatus::New;
	motionPlan = NULL;
	haltRequested = false;

	logfileName = "Data_" + jointModel->name + ".csv";
				
	TimeUtil::setNow(controllerStartTime);

	cSensorAngle = 0;
	cRawSensorAngle = 0;
	cVelocity = 0;
	cMotorTorque = 0;
	cDynamicTorque = 0;
	cTargetAngle = 0;
	cTargetAngleDistance =0;
	cTargetVelocity = 0;

	cTargetVoltage = 0;
	cAppliedVoltage = 0;
	nVoltage = 0;
	nTargetVoltage = 0;
	nAppliedVoltage = 0;
	lDynamicPositionError = 0;

	cTime = 0;
	lTime = 0;
	nDriverMode = DriverMode::Coast;	
	cDriverCommand = 0xFF;
	
	isControlTorqueValid = false;

	flipRequestedByUser = false;
	requestedFlipDirection = 0;
	expectedRotationalStopDirection = 0;

	speedControlProportionalGain = servoModel->controllerConfig.speedControlProportionalGain;
	speedControlIntegralGain = servoModel->controllerConfig.speedControlIntegralGain;

	if (Configuration::CsvLoggingEnabled)
	{		
		writeLogHeader();
	}
	
}

void PredictiveJointController::prepare()
{
	if (ServoUtils::validateAndPrintJointFunction(bus, jointModel))
	{
		jointStatus = JointStatus::Ready;
	}
	else
	{
		jointStatus = JointStatus::Paused;
	}
}

void PredictiveJointController::enable()
{
	if (jointStatus == JointStatus::Ready)
	{
		setCurrentState();
		motionPlan = shared_ptr<MotionPlan>(new MotionPlan(cSensorAngle));
		motionPlan->startNow();

		motionPlanComplete = true;
		staticControlMode = StaticControlMode::Holding;
		jointStatus = JointStatus::Active;		
				
		std::string upName = std::string(jointModel->name);
		std::transform(upName.begin(), upName.end(),upName.begin(), ::toupper);		
		cout << "JOINT " << upName << " IS ONLINE" << endl;

		TimeUtil::setNow(enableTime);
	}
	else
	{
		cout << "Joint must be in 'Ready' state to activate." << endl;
	}
}

void PredictiveJointController::pause()
{
	if (jointStatus == JointStatus::Active)
	{
		emergencyHalt("Paused");
		if (!haltRequested && jointStatus == JointStatus::Error)
			jointStatus = JointStatus::Paused;
	}
}

void PredictiveJointController::disable()
{
	emergencyHalt("Disabled by user");
}

void PredictiveJointController::executeMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan)
{	
	validateMotionPlan(requestedMotionPlan);
	
	if (this->motionPlan != NULL)
		motionPlan.reset();		
	
	this->motionPlan = requestedMotionPlan;

	motionPlanComplete = false;	
	isTorqueEstimateValid = false;
	isControlTorqueValid = false;
	dynamicControlMode = DynamicControlMode::Starting;
	startModeIndex = 0;

	//Make sure speed controller starts off in adjusting state
	speedControlState = SpeedControlState::Adjusting;
	lDynamicPositionError = 0;

	while (rawSensorAngleHistory.size() > 1)
		rawSensorAngleHistory.pop_front();
}

void PredictiveJointController::validateMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan)
{
	if (!(jointStatus == JointStatus::Active || jointStatus == JointStatus::Paused))
		throw std::runtime_error("Joint must be in Active state to execute a motion plan");
	
	if (requestedMotionPlan->finalAngle < jointModel->minAngle || requestedMotionPlan->finalAngle > jointModel->maxAngle)
	{
		throw std::runtime_error("Motion plan angle exceeds joint range");
	}
}



//bool PredictiveJointController::handleUserRequests()
//{
//	bool handled = false;
//	if (readyForCommand)
//	{
//		if (flipRequestedByUser)
//		{
//			flipRequestedByUser = false;
//			requestedFlipDirection = 0;
//			handled = true;
//		}
//		else if (patternRequestedByUser)
//		{
//			patternRequestedByUser = false;
//			list<double> * executePattern = new list<double>(requestedVoltagePattern.begin(),requestedVoltagePattern.end());
//
//			externalController = [this,executePattern]()
//			{
//				if (!executePattern->empty())
//				{
//					commandDriver(executePattern->front(),DriverMode::ConstantVoltage);			
//					executePattern->pop_front();
//				}
//				else 
//				{
//					//delete executePattern;
//					//executePattern = NULL;
//					readyForCommand = true;
//					commandDriver(0,DriverMode::Brake);			
//				}
//			};
//		
//			commandDriver(0,DriverMode::Brake);		
//			controlMode = ControlMode::External;
//			handled = true;
//		} 
//		if (handled) readyForCommand = false;
//	}
//	return handled;
//}

void PredictiveJointController::writeLogHeader()
{
	stringstream ss;
	ss <<
		"Time"				<< Configuration::CsvSeparator <<
		"JointStatus"		<< Configuration::CsvSeparator <<
		"RawSensorAngle"	<< Configuration::CsvSeparator <<
		"SensorAngle"		<< Configuration::CsvSeparator <<
		"TargetAngle"		<< Configuration::CsvSeparator <<
		"Velocity"			<< Configuration::CsvSeparator <<
		"VelocityR2"		<< Configuration::CsvSeparator << 
		"TargetVelocity"	<< Configuration::CsvSeparator <<
		"PredictedTorque"	<< Configuration::CsvSeparator <<
		"ControlTorque"		<< Configuration::CsvSeparator <<
		"EffectiveVoltage"	<< Configuration::CsvSeparator <<
		"AverageVoltaged"	<< Configuration::CsvSeparator <<
		"AppliedVoltage"	<< Configuration::CsvSeparator <<
		"ControlMode"		<< Configuration::CsvSeparator <<
		"SecondaryState"	<< Configuration::CsvSeparator <<
		"SensorWriteTime"	<< Configuration::CsvSeparator <<
		"SensorReadTime"	<< Configuration::CsvSeparator <<
		"DriverWriteTime"	<< Configuration::CsvSeparator <<
		"StableTorque"		<< Configuration::CsvSeparator << 
		"BusSelectTime"		<< Configuration::CsvSeparator << 
		"FileWriteTime"		<< Configuration::CsvSeparator << 
		"QuadRegAccel"		<< Configuration::CsvSeparator << 
		"PlanAccel"			<< Configuration::CsvSeparator <<
		"PlanVelocity"		<< Configuration::CsvSeparator << 
		"ExpectedVelocity"	<< Configuration::CsvSeparator << endl;

	AsyncLogger::getInstance().postLogTask(logfileName,ss.str());
}


void PredictiveJointController::logState()
{
	if (TimeUtil::timeSince(enableTime) < samplePeriod*3.0) return;

	timespec logStart;
	TimeUtil::setNow(logStart);

	const double torqueScale = 100.0;
	double writeTime = TimeUtil::timeSince(controllerStartTime);
	stringstream ss;

	ss
		<< cTime			<< Configuration::CsvSeparator
		<< jointStatus		<< Configuration::CsvSeparator
		<< cRawSensorAngle	<< Configuration::CsvSeparator
		<< cSensorAngle		<< Configuration::CsvSeparator
		<< cTargetAngle		<< Configuration::CsvSeparator
		<< cVelocity		<< Configuration::CsvSeparator
		<< cVelocityApproximationError << Configuration::CsvSeparator
		<< cTargetVelocity	<< Configuration::CsvSeparator
		<< cPredictedTorque*torqueScale << Configuration::CsvSeparator
		<< cControlTorque*torqueScale << Configuration::CsvSeparator
		<< cVoltage			<< Configuration::CsvSeparator
		<< nTargetVoltage	<< Configuration::CsvSeparator
		<< cAppliedVoltage	<< Configuration::CsvSeparator;

	
	if (dynamicControl)
	{	
		if (dynamicControlMode == DynamicControlMode::Travelling)
		{			
			ss << 1	<< Configuration::CsvSeparator;
			ss << speedControlState;
		}
		else
		{
			ss << 2	<< Configuration::CsvSeparator;
			ss << dynamicControlMode;
		}
	}
	else 
	{
		if (staticControlMode == StaticControlMode::Stepping)
		{			
			ss << -2	<< Configuration::CsvSeparator;
			ss << steppingState;
		}
		else if (staticControlMode == StaticControlMode::Holding)
		{			
			ss << -1	<< Configuration::CsvSeparator;
			ss << 0;
		}
	}


	ss 
		<< Configuration::CsvSeparator 
		<< cSensorWriteTime		<< Configuration::CsvSeparator 
		<< cSensorReadTime		<< Configuration::CsvSeparator 
		<< cDriverWriteTime		<< Configuration::CsvSeparator 
		<< stableTorqueEstimate*100.0 << Configuration::CsvSeparator
		<< cBusSelectTime		<< Configuration::CsvSeparator
		<< writeTime			<< Configuration::CsvSeparator
		<< cQuadRegAcceleration	<< Configuration::CsvSeparator
		<< cTargetAcceleration	<< Configuration::CsvSeparator
		<< cPlanTargetVelocity	<< Configuration::CsvSeparator 
		<< servoModel->getSpeedForTorqueVoltage(cControlTorque,cVoltage);

	ss << endl;

	AsyncLogger::getInstance().postLogTask(logfileName,ss.str());

	TimeUtil::assertTime(logStart,jointModel->name + ".logState()");

}

double PredictiveJointController::getMaxVelocity()
{
	const double AverageTorque = 0;

	return servoModel->getSpeedForTorqueVoltage(AverageTorque,DRV8830::stepsToVoltage(getMaxVoltageSteps()));
}

double PredictiveJointController::getMaxAcceleration()
{
	return config->maxAcceleration;
}

JointModel * PredictiveJointController::getJointModel()
{
	return jointModel;
}

double PredictiveJointController::getCurrentAngle()
{
	return cSensorAngle;
}

double PredictiveJointController::getMaxVoltageSteps()
{
	return std::min<double>(DRV8830::MaxVoltageStep,DRV8830::voltageToSteps(servoModel->maxDriverVoltage));
}

string PredictiveJointController::getJointStatusText()
{
	switch (jointStatus)
	{
	case JointStatus::Active:
		return "Active";
	case JointStatus::Error:
		return "Error";
	case JointStatus::New:
		return "New";
	case JointStatus::Ready:
		return "Ready";
	case JointStatus::Paused:
		return "Paused";
	default:
		return "UNKNOWN STATUS";
	}
}

JointStatus PredictiveJointController::getJointStatus()
{
	return this->jointStatus;
}

bool PredictiveJointController::jointReadyForCommand()
{
	return false; //return (readyForCommand && (controlMode == ControlMode::Hold || controlMode == ControlMode::External));
}

void PredictiveJointController::requestFlip(int direction, double voltage)
{
	//if (std::abs(direction) < 20)
	//{
	//	if (controlMode == ControlMode::Hold)
	//	{
	//		flipRequestedByUser = true;
	//		requestedFlipDirection = direction;
	//		if (voltage > 0)
	//			requestedFlipVoltage = DRV8830::getNearestVoltage(voltage);
	//		else
	//			requestedFlipVoltage = LostMotionFlipVoltage;

	//	}
	//	else
	//	{
	//		stringstream ss;
	//		ss << "Flip can only be requested in ControlMode::Hold. Current mode = " << controlMode;
	//		throw std::runtime_error(ss.str());
	//	}
	//}
	//else
	//{
	//	throw std::runtime_error("Requested flip direction is invalid.");
	//}
	//readyForCommand = false;
}

void PredictiveJointController::requestPattern(vector<double> pattern)
{
	if (pattern.empty())
		throw std::runtime_error("Requested pattern is empty");

	requestedVoltagePattern = pattern;
	patternRequestedByUser = true;
}










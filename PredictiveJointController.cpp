#include "PredictiveJointController.hpp"



using namespace std;

void PredictiveJointController::init()
{
	filter_lowpass_position	=	new LowpassFilter(config->get("SensorFilters.LowpassPositionFilter.TimeConstant"));

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
	lVelocityError = 0;
	cTime = 0;
	lTime = 0;
	nDriverMode = DriverMode::Coast;	
	cDriverCommand = 0xFF;
	cRevolutionCount = 0;
	cRunTime = 0;
	isControlTorqueValid = false;

	speedControlProportionalGain = servoModel->controllerConfig.speedControlProportionalGain;
	speedControlIntegralGain = servoModel->controllerConfig.speedControlIntegralGain;

	if (Configuration::CsvLoggingEnabled)
	{		
		writeLogHeader();
	}
	
	TimeUtil::setNow(controllerStartTime);
}

void PredictiveJointController::prepare()
{
	if (ServoUtils::validateAndPrintJointFunction(bus[servoModel->driverBus],bus[servoModel->sensorBus], jointModel))
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

void PredictiveJointController::joinMotionPlan(std::shared_ptr<MotionPlan> newMotionPlan)
{
	//If current plan is already done, just start the new one
	if (motionPlanComplete || motionPlan == NULL || TimeUtil::timeUntil(motionPlan->endTime) < 0) 
	{
		executeMotionPlan(newMotionPlan);
	}
	else
	{
		validateMotionPlan(newMotionPlan);
		//double t = TimeUtil::timeBetween(motionPlan->startTime,newMotionPlan->startTime);
		//
		//double xJoin = newMotionPlan->x(0);
		//double dxJoin = newMotionPlan->dx(0);
		//
		//double xError = abs(xJoin - cSensorAngle);
		//double dxError = abs(dxJoin - cVelocity);

		//if (xError > 100 || dxError > 100) throw std::runtime_error("Error between new plan and current state is too high");
			
		this->motionPlan = newMotionPlan;
		speedControlState = SpeedControlState::Adjusting;
		dynamicControlMode = DynamicControlMode::Starting;
		lDynamicPositionError = 0;
		motionPlanComplete = false;
	}
}

void PredictiveJointController::executeMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan)
{	
	validateMotionPlan(requestedMotionPlan);
		
	this->motionPlan = requestedMotionPlan;

	motionPlanComplete = false;
	isControlTorqueValid = false;
	dynamicControlMode = DynamicControlMode::Starting;

	//Make sure speed controller starts off in adjusting state
	speedControlState = SpeedControlState::Adjusting;
	lDynamicPositionError = 0;

	//while (rawSensorAngleHistory.size() > 1)
	//	rawSensorAngleHistory.pop_front();
}

void PredictiveJointController::validateMotionPlan(std::shared_ptr<MotionPlan> requestedMotionPlan)
{
	if (TimeUtil::timeUntil(requestedMotionPlan->startTime) > 0.2)
	{
		stringstream ss;
		ss << "Plan starts " << TimeUtil::timeUntil(requestedMotionPlan->startTime)  << " s in the future";
		throw std::runtime_error(ss.str());
	}
	
	if (TimeUtil::timeUntil(requestedMotionPlan->endTime) < -0.2)
	{
		stringstream ss;
		ss << "Plan ended " << TimeUtil::timeSince(requestedMotionPlan->endTime)  << " s ago";
		throw std::runtime_error(ss.str());
	}

	if (!(jointStatus == JointStatus::Active || jointStatus == JointStatus::Paused))
		throw std::runtime_error("Joint must be in Active state to execute a motion plan");
	
	if (!jointModel->continuousRotation && (requestedMotionPlan->finalAngle < jointModel->minAngle || requestedMotionPlan->finalAngle > jointModel->maxAngle))
	{
		stringstream ss;
		ss << jointModel->name << " - Motion plan angle out of range: " << AS5048::stepsToDegrees(requestedMotionPlan->finalAngle);
		throw std::runtime_error(ss.str());
	}
}

void PredictiveJointController::writeLogHeader()
{
	if (Configuration::FastLogging)
	{
		//
	}	
	else
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
			"TargetVoltage"		<< Configuration::CsvSeparator <<
			"AppliedVoltage"	<< Configuration::CsvSeparator <<
			"ControlMode"		<< Configuration::CsvSeparator <<
			"SecondaryState"	<< Configuration::CsvSeparator <<
			"SensorWriteTime"	<< Configuration::CsvSeparator <<
			"SensorReadTime"	<< Configuration::CsvSeparator <<
			"DriverWriteTime"	<< Configuration::CsvSeparator <<
			"MotorTorque"		<< Configuration::CsvSeparator <<
			"RunTime"			<< Configuration::CsvSeparator <<
			"QuadRegAccel"		<< Configuration::CsvSeparator << 
			"PlanAccel"			<< Configuration::CsvSeparator <<
			"PlanVelocity"		<< Configuration::CsvSeparator << 
		"ExpectedVelocity"	<< Configuration::CsvSeparator << endl;
		AsyncLogger::getInstance().postLogTask(logfileName,ss.str());
	}

}

void PredictiveJointController::writeHistoryToLog()
{
	stringstream ss;
	ss << "Time,SensorAngle,TargetAngle,Velocity,TargetVelocity,PlanVelocity" << endl;

	while (!dataHistory.empty())
	{
		DataFrame frame = dataHistory.front();
		dataHistory.pop_front();

		ss
			<< frame.Time		<< Configuration::CsvSeparator
			<< frame.Angle		<< Configuration::CsvSeparator
			<< frame.TargetAngle	<< Configuration::CsvSeparator
			<< frame.Velocity		<< Configuration::CsvSeparator
			<< frame.TargetVelocity	<< Configuration::CsvSeparator
			<< frame.PlanVelocity << Configuration::CsvSeparator << endl;
		
	}
	AsyncLogger::getInstance().postLogTask(logfileName,ss.str());
}


void PredictiveJointController::logState()
{
	if (TimeUtil::timeSince(enableTime) < Configuration::SamplePeriod*3.0) return;

	if (Configuration::FastLogging)
	{
		dataHistory.push_back(DataFrame(cTime,cSensorAngle,cVelocity,cTargetAngle,cTargetVelocity,cPlanTargetVelocity));	
	}
	else
	{
		timespec logStart;
		TimeUtil::setNow(logStart);

		const double torqueScale = 100.0;
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
			else //if (staticControlMode == StaticControlMode::Holding)
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
			<< cMotorTorque*torqueScale << Configuration::CsvSeparator
			<< cRunTime				<< Configuration::CsvSeparator
			<< cQuadRegAcceleration	<< Configuration::CsvSeparator
			<< cTargetAcceleration	<< Configuration::CsvSeparator
			<< cPlanTargetVelocity	<< Configuration::CsvSeparator 
			<< servoModel->getSpeedForTorqueVoltage(cControlTorque,cVoltage);

		ss << endl;

		AsyncLogger::getInstance().postLogTask(logfileName,ss.str());

		TimeUtil::assertTime(logStart,jointModel->name + ".logState()",0.01);
	}

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

double PredictiveJointController::getAngleSetpoint()
{
	return cTargetAngle;
}

double PredictiveJointController::getCurrentVelocity()
{
	return cVelocity;
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


bool PredictiveJointController::isDynamicMode()
{
	return dynamicControl;
}



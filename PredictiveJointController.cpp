#include "PredictiveJointController.hpp"

namespace ControllerConfiguration 
{
	const double MinimumStopTime = 0; //seconds
	const double VelocityZeroThreshold = 30;
	const double MinTorqueScale = 1.2;
	const double MinContinuousApproachDistance = 100;
	
	const double BaseApproachVoltage = 0.68;
	const double MaximumStopTimeScale = 1.5;
	const double MinApproachSpeed = 400; //steps/second

	const double SteppingModeReadDelay = 0.03; //seconds
	const double MaxMotionForValidStep = 5;//steps
	const double BaseStepVoltage = 1.04; //V

	const double MaxSetpointError = 5; //steps
	const double MinDisturbanceError = 100;	
	
	//Speed Control
	const double MinSpeedControlMeasureDelay = 0.03; //seconds
	const double MaxSpeedControlMeasureDelay = 0.08; //seconds
	const double MinSpeedControlDistance = 200;
	const double MinVelocityRValue = 0.95;
	const double BaseSpeedControlMeasureTorque = 0.3;
	const double MaxVelocitySetpointError = 20; //steps/second
	//const double MaxStableVelocitySetpointError = 80; //steps/second

	const double TorqueSMAFilterWindowSize = 10;
};

using namespace std;
using namespace ControllerConfiguration;

void PredictiveJointController::init()
{
	const double TorqueFilterTimeConstant = 0.05;
	const double PositionMovingAverageWindowSize = 4;
	const double SpeedMovingAverageWindowSize = 4;
	const double SpeedFilterTimeConstant = 0.05;
	
	filter_lowpass_for_motorTorque =	new LowpassFilter(TorqueFilterTimeConstant);
	filter_sma_angle_for_position =		new SimpleMovingAverage(PositionMovingAverageWindowSize);
	filter_sma_angle_for_speed =		new SimpleMovingAverage(SpeedMovingAverageWindowSize);
	filter_lowpass_speed =				new LowpassFilter(SpeedFilterTimeConstant);
	filter_sma_for_speedController_motorTorque =		new SimpleMovingAverage(TorqueSMAFilterWindowSize);

	voltageConverter = new TimeMultiplexedVoltageConverter(2);

	speedControlState = SpeedControlState::Measuring;
	positionControlState = PositionControlState::Missed;
	controlMode = ControlMode::Disabled;
	jointStatus = JointStatus::New;
	motionPlan = NULL;

	if (Configuration::CsvLoggingEnabled)
	{
		csvLog.open("Data_" + jointModel->name + ".csv");
		csvLog <<
			"Time"				<< Configuration::CsvSeparator <<
			"JointStatus"		<< Configuration::CsvSeparator <<
			"RawSensorAngle"	<< Configuration::CsvSeparator <<
			"SensorAngle"		<< Configuration::CsvSeparator <<
			"TargetAngle"		<< Configuration::CsvSeparator <<
			"Velocity"			<< Configuration::CsvSeparator <<
			"VelocityR2"		<< Configuration::CsvSeparator << 
			"TargetVelocity"	<< Configuration::CsvSeparator <<
			"DisturbanceTorque" << Configuration::CsvSeparator <<
			"MotorTorque"		<< Configuration::CsvSeparator <<
			"EffectiveVoltage"	<< Configuration::CsvSeparator <<
			"AverageVoltaged"	<< Configuration::CsvSeparator <<
			"AppliedVoltage"	<< Configuration::CsvSeparator <<
			"ControlMode"		<< Configuration::CsvSeparator <<
			"SecondaryState"		<< endl;
	}
	
	lTime = 0;
	nVoltage = 0;
	nTargetVoltage = 0;
	nAppliedVoltage = 0;
	nDriverMode = DriverMode::Coast;
	
	MathUtil::setNow(startTime);	
}

void PredictiveJointController::prepare()
{
	if (ServoUtils::validateAndPrintJointFunction(bus, jointModel))
	{
		jointStatus = JointStatus::Ready;
	}
	else
	{
		cout << "Joint initialization failed." << endl;
		jointStatus = JointStatus::Error;
	}
}

void PredictiveJointController::enable()
{
	if (jointStatus == JointStatus::Ready)
	{
		std::string upName = std::string(jointModel->name);
		std::transform(upName.begin(), upName.end(),upName.begin(), ::toupper);

		setpointHoldAngle = MathUtil::offsetAngleSteps(getSensorAngleRegisterValue(),jointModel->sensorZeroPosition);
		jointStatus = JointStatus::Active;
		controlMode = ControlMode::Hold;

		cout << "JOINT " << upName << " IS ONLINE" << endl;
	}
	else
	{
		cout << "Joint must be in 'Ready' state to activate." << endl;
	}
}

void PredictiveJointController::disable()
{
	controlMode = ControlMode::Disabled;
}

void PredictiveJointController::executeMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan)
{
	cout << "Starting new motion plan. " << endl;
	validateMotionPlan(requestedMotionPlan);
	
	if (this->motionPlan != NULL)
		motionPlan.reset();		
	
	this->motionPlan = requestedMotionPlan;
	controlMode = ControlMode::SpeedControl;
	speedControlState = SpeedControlState::Measuring;	
	speedControlStableTorque = 0;
	MathUtil::setNow(planStartTime);
}

void PredictiveJointController::validateMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan)
{
	if (jointStatus != JointStatus::Active)
		throw std::runtime_error("Joint must be in Active state to execute a motion plan");
	
	if (requestedMotionPlan->finalAngle < jointModel->minAngle || requestedMotionPlan->finalAngle > jointModel->maxAngle)
	{
		throw std::runtime_error("Motion plan angle exceeds joint range");
	}
}


double timeSince(timespec & sinceTime){
	timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	return MathUtil::getTimeDelta(sinceTime,now);
}

int PredictiveJointController::getSensorAngleRegisterValue()
{
	int result = -1;
	try 
	{
		bus->selectAddress(servoModel->sensorAddress);
		result = AS5048::getSensorAngleSteps(bus);
	}
	catch (std::runtime_error & e)
	{
		emergencyHalt("Error reading sensor. ");
		cout << "Exception in getSensorAngleRegisterValue(): " <<  e.what() << endl;
	}
	return result;
}

void PredictiveJointController::emergencyHalt(std::string reason)
{
	try
	{
		controlMode = ControlMode::Disabled;
		//Attempt shutdown 
		bus->selectAddress(servoModel->driverAddress);
		DRV8830::writeVoltageMode(bus, 0, DriveMode::OFF);		
		jointStatus = JointStatus::Error;
		cout << "Joint " << jointModel->name << " is now offline" << endl;
		cout << "Reason for shutdown: " << reason << endl;
	}
	catch (std::runtime_error & e)
	{
		cout << "HALP I CANNOT STOPS I HAZ EXCPTIONZ " << e.what() << endl;
	}
}

void PredictiveJointController::setCurrentState()
{
	const int HistorySize = 10;
	const int VoltageHistorySize = 10;

	//Push current to last
	lRawSensorAngle = cRawSensorAngle;
	lTime = cTime;
	
	//Carry next to current
	cDriverCommanded = false;
	
	cVoltage = nVoltage;
	cAppliedVoltage = nAppliedVoltage;
	cTargetVoltage = nTargetVoltage;
	cDriverMode = nDriverMode;
	
	//Set new values
	cNonZeroOffsetSensorAngle = getSensorAngleRegisterValue();
	cRawSensorAngle = correctAngleForDiscreteErrors(MathUtil::offsetAngleSteps(cNonZeroOffsetSensorAngle,jointModel->sensorZeroPosition));
	
	cTime = timeSince(startTime);
	
	appliedVoltageHistory.push_back(cAppliedVoltage);
	if (appliedVoltageHistory.size() > VoltageHistorySize) appliedVoltageHistory.pop_front();

	rawSensorAngleHistory.push_back(make_pair(cTime,cRawSensorAngle));
	if (rawSensorAngleHistory.size() > HistorySize) rawSensorAngleHistory.pop_front();
	
	
	setApproximateSpeed(rawSensorAngleHistory);
	cSensorAngle = filterAngle(cRawSensorAngle); //Filter for position

	double velocityMag;
	if (controlMode == ControlMode::Hold)
	{
		if (motionPlan == NULL)
			cTargetAngle = setpointHoldAngle;
		else
			cTargetAngle = motionPlan->finalAngle;

		velocityMag = 0;
	}
	else
	{
		cTargetAngle = motionPlan->finalAngle;
		velocityMag = std::abs(motionPlan->getSpeedAtTime(MathUtil::timeSince(planStartTime)));
	}

	cTargetAngleDistance = AS5048::getAngleError(cSensorAngle,cTargetAngle);
	double targetDirection = MathUtils::sgn<double>(cTargetAngleDistance);
	cTargetVelocity = velocityMag*targetDirection;

	if (speedControlStableTorque == 0)
		speedControlStableTorque = targetDirection * BaseSpeedControlMeasureTorque;

	//Static torque resulting from current arm pose. Zero for now.
	//TODO: Also need to consider torque that results from angular acceleration
	cModelJointTorque = 0;
			
	//Torque experienced by the motor, over last N frames.
	cAverageVoltage = std::accumulate(appliedVoltageHistory.begin(), appliedVoltageHistory.end(), 0.0) / appliedVoltageHistory.size();
	cMotorTorque = servoModel->getTorqueForVoltageSpeed(cAverageVoltage, cVelocity);	

	cDisturbanceTorque = cMotorTorque; // filter_lowpass_for_motorTorque->next(cMotorTorque - cModelJointTorque);
}


void PredictiveJointController::setApproximateSpeed(std::list<std::pair<double, double> > history)
{
	double guess = computeSpeed(history.back().second);
	double slope, intercept, rValue;
	if (MathUtil::linearRegression(history,slope,intercept,rValue))
	{
		cVelocity = slope;
		cVelocityApproximationError = std::pow(rValue,2);
	}
	else
	{
		cVelocity = guess;
		cVelocityApproximationError = 0;
	}
}

//Check if angle is ~64 steps away from the previous value. If it is, then then 6th angle bit may be flip-flopped.
//This check is only done if it is enabled for the joint.
double PredictiveJointController::correctAngleForDiscreteErrors(double rawAngle)
{
	const double MaxCorrectedOffsetFromPrevious = 7.0;
	const double MinRValueForDisplacementPrediction = 0.8;

	double predictedDisplacement = 0;
	if (cVelocityApproximationError > MinRValueForDisplacementPrediction)
	{
		predictedDisplacement = cVelocity*samplePeriod;
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


/*
 Determining speed is non-trivial due to sensor noise.
 Sensor noise can be removed with a low-pass filter, but this may not be sufficient as the frequency
 of the noise is close to the frequency of the actual velocity changes.
 
 More investigation is needed to determine the optimal filtering strategy.
 Minimizing delay is critical to obtain smooth motion.
 */
double PredictiveJointController::computeSpeed(double rawSensorAngle)
{
	//	filter_lowpass_angle_for_speed
	//	filter_sma_angle_for_speed
	
	filter_sma_angle_for_speed->add(rawSensorAngle);
	double filteredAngle = filter_sma_angle_for_speed->avg();
	
	double delta = MathUtil::subtractAngles(filteredAngle,lFilteredAngleForSpeed,MathUtil::PI_STEPS);
	lFilteredAngleForSpeed = filteredAngle;

	//lowpass filter on speed
	//delta = filter_lowpass_speed->next(delta);
	
	double velocity = delta/(cTime - lTime);

	return velocity;
}

double PredictiveJointController::filterAngle(int rawSensorAngle)
{
	filter_sma_angle_for_position->add(rawSensorAngle);
	return filter_sma_angle_for_position->avg();
}

void PredictiveJointController::performSafetyChecks()
{
	if (cRawSensorAngle < jointModel->minAngle || cRawSensorAngle > jointModel->maxAngle || 
		cSensorAngle < jointModel->minAngle || cSensorAngle > jointModel->maxAngle)
	{
		emergencyHalt("Safety check failed");

		cout << "Error! Joint " << jointModel->name << " has exceeded angle limits. " << endl;
		printState();
	}
}

void PredictiveJointController::run()
{
		
	if (jointStatus != JointStatus::Active) 
	{
		return;
	}
	
	if (controlMode == ControlMode::Disabled)
	{
		emergencyHalt("Joint running with control disabled");
		return;
	}

	setCurrentState();		
	performSafetyChecks();

	if (!cDriverCommanded && controlMode == ControlMode::Hold) 
	{
		if (std::abs(cTargetAngleDistance) > MinDisturbanceError)
		{
			controlMode = ControlMode::PositionControl;
			positionControlState = PositionControlState::Missed;
		}
		else
			commandDriver(0,DriverMode::Brake);
	}
	
	if (!cDriverCommanded && controlMode == ControlMode::SpeedControl) {
		doSpeedControl();
	}

	if (!cDriverCommanded && controlMode == ControlMode::PositionControl) {
		doPositionControl();		
	}
	
	if (!cDriverCommanded && controlMode == ControlMode::StepControl) {
		doStepControl();
	}

	commitCommands();
	logState();
}


void PredictiveJointController::doSpeedControl()
{
	if (std::abs(cTargetAngleDistance) < MinSpeedControlDistance)
	{
		controlMode = ControlMode::PositionControl;
		positionControlState = PositionControlState::Stabilizing;
	}
	else 
	{
		//Velocity is accurate		
		if (cVelocityApproximationError > MinVelocityRValue)
		{
			filter_sma_for_speedController_motorTorque->add(cMotorTorque);
			//Reached target speed, minimize voltage adjustments 
			if (MathUtil::timeSince(speedControlMeasureStart) < MinSpeedControlMeasureDelay)
			{
				speedControlState = SpeedControlState::Adjusting;				
				commandDriver(servoModel->getVoltageForTorqueSpeed(speedControlStableTorque,cTargetVelocity),DriverMode::TMVoltage);	
			}
			else if (std::abs(cVelocity - cTargetVelocity) < MaxVelocitySetpointError)
			{						
				speedControlState = SpeedControlState::Stable;				
				commandDriver(servoModel->getVoltageForTorqueSpeed(speedControlStableTorque,cTargetVelocity),DriverMode::TMVoltage);	
			}
			//Stable speed, but wrong value. Adjust voltage
			else 
			{
				MathUtil::setNow(speedControlMeasureStart);
				speedControlStableTorque = filter_sma_for_speedController_motorTorque->avg();
				commandDriver(servoModel->getVoltageForTorqueSpeed(speedControlStableTorque,cTargetVelocity),DriverMode::TMVoltage);	
				speedControlState = SpeedControlState::Adjusting;										
			}
		}
		//Unstable velocity, attempt to stabilize using last torque reading
		else
		{				
			speedControlState = SpeedControlState::Measuring;			
			commandDriver(servoModel->getVoltageForTorqueSpeed(speedControlStableTorque,cTargetVelocity),DriverMode::TMVoltage);			
		}
	}
}

void PredictiveJointController::doPositionControl()
{
	//Approaching at constant speed, or just beginning approach
	if (positionControlState == PositionControlState::Approaching || positionControlState == PositionControlState::Stabilizing) 
	{
		//Moving, try to hit setpoint by braking at the right time
		if (std::abs(cVelocity) > VelocityZeroThreshold) 
		{
			double predictedAngle = cSensorAngle + (cVelocity*servoModel->sensorDelay); //Try to predict error caused by sensor delay
			double predictedDistance = MathUtil::subtractAngles(cTargetAngle, predictedAngle, MathUtil::PI_STEPS);
			double timeToSetpoint = predictedDistance / cVelocity;

			double stopIn = timeToSetpoint - servoModel->driverDelay;

			//Too late to stop, brake
			if (stopIn < MinimumStopTime)
			{
				positionControlState = PositionControlState::Missed;
				commandDriver(0,DriverMode::Brake);
			}		
			else if (stopIn < (MaximumStopTimeScale*samplePeriod))
			{
				positionControlState = PositionControlState::Stopping;
				commandDriver(0,DriverMode::Brake);
			}
			else 
			{
				//Keep steady and wait
				if (positionControlState == PositionControlState::Stabilizing) 
				{
					double approachVoltage =  servoModel->getVoltageForTorqueSpeed(speedControlStableTorque,cTargetVelocity); //BaseApproachVoltage*MathUtils::sgn<double>(cTargetAngleDistance);
					positionControlState = PositionControlState::Approaching;	
					commandDriver(approachVoltage,DriverMode::ConstantVoltage);
				}
				//Maintain voltage
				else if (positionControlState == PositionControlState::Approaching)
				{					
					commandDriver(cVoltage,DriverMode::ConstantVoltage);
				}
			}
		}
		//If not moving, then either complete, or transition to stepping mode
		else 
		{
			if (std::abs(cTargetAngleDistance) < MaxSetpointError)
			{
				controlMode = ControlMode::Hold;
				commandDriver(0,DriverMode::Brake);
			}
			else
			{
				positionControlState = PositionControlState::Missed;
			}
		}	
	}
	else if (positionControlState == PositionControlState::Stopping)
	{
		if (std::abs(cVelocity) < VelocityZeroThreshold) 
		{
			if (std::abs(cTargetAngleDistance) < MaxSetpointError)
			{
				controlMode = ControlMode::Hold;
				commandDriver(0,DriverMode::Brake);
			}
			else
			{
				positionControlState = PositionControlState::Missed;
			}
		}
		else
		{
			commandDriver(0,DriverMode::Brake);
		}
	}


	//Could transition states in above block without generating a command
	if (!cDriverCommanded && positionControlState == PositionControlState::Missed) 
	{
		if (std::abs(cTargetAngleDistance) < MinContinuousApproachDistance)
		{
			controlMode = ControlMode::StepControl;
			executeStep(BaseStepVoltage*MathUtils::sgn<double>(cTargetAngleDistance),1);
		}
		else
		{
			double approachVoltage = BaseApproachVoltage*MathUtils::sgn<double>(cTargetAngleDistance);
			//double minVoltage = servoModel->getVoltageForTorqueSpeed(cMotorTorque*MinTorqueScale,MathUtils::sgn<double>(cTargetAngleDistance)*MinApproachSpeed);
			positionControlState = PositionControlState::Approaching;										
			commandDriver(approachVoltage,DriverMode::ConstantVoltage);
		}
	}
}

void PredictiveJointController::doStepControl()
{
	if (std::abs(cTargetAngleDistance) > MaxSetpointError) 
	{
		switch (steppingState)
		{
		case SteppingState::Energizing:
			commandDriver(stepVoltages.at(stepVoltageIndex++),DriverMode::ConstantVoltage);
			if (stepVoltageIndex >= stepVoltages.size())
			{
				steppingState = SteppingState::Braking;
			}
			break;
		case SteppingState::Braking:
			steppingState = SteppingState::Reading;
			MathUtil::setNow(readDelayStart);
			commandDriver(0,DriverMode::Coast);
			break;
		case SteppingState::Reading:
			if (MathUtil::timeSince(readDelayStart) > SteppingModeReadDelay) 
			{
				int stepDirection = MathUtils::sgn<double>(cTargetAngleDistance);

				//Overshoot, reverse direction and step again.
				if (MathUtils::sgn<double>(cTargetAngleDistance) != MathUtils::sgn<double>(stepInitialTargetDistance))
				{						
					executeStep(BaseStepVoltage*stepDirection,1);
				}
				//Didn't go far enough to count as motion, or went backwards. Step again with higher voltage
				else if (std::abs(cTargetAngleDistance - stepInitialTargetDistance) < MaxMotionForValidStep) 
				{
					int length = 1;
					int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltageIntegral));
					if (vSteps >= DRV8830::MaxVoltageStep)
						length = 2;
					else
						vSteps++;

					executeStep(DRV8830::stepsToVoltage(vSteps)*stepDirection,length);
				}
				//Didn't make it, repeat step at same voltage
				else 
				{
					int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltageIntegral));
					executeStep(DRV8830::stepsToVoltage(vSteps)*stepDirection,1);
				}
			}
			else
			{
				commandDriver(0,DriverMode::Brake);
			}
			break;
		}
	}
	else {
		controlMode = ControlMode::Hold;
		commandDriver(0,DriverMode::Brake);
	}
}

void PredictiveJointController::executeStep(double voltage, int energizeLength) 
{
	stepInitialTargetDistance = MathUtil::subtractAngles(cTargetAngle,cSensorAngle,MathUtil::PI_STEPS);

	stepVoltages.clear();
	
	for (int i=0;i<energizeLength;i++)
		stepVoltages.push_back(voltage);

	stepVoltages.push_back(0);
	
	stepVoltageIntegral = std::accumulate(stepVoltages.begin(), stepVoltages.end(), 0.0);

	stepStartPosition = cSensorAngle;
	stepExpectedDirection = MathUtils::sgn<double>(voltage); //Assuming direction is the same sign as voltage
	
	stepVoltageIndex = 0;
	commandDriver(stepVoltages.at(stepVoltageIndex++),DriverMode::ConstantVoltage);
	
	if (stepVoltageIndex < stepVoltages.size())
		steppingState = SteppingState::Energizing;
	else
		steppingState = SteppingState::Braking;
}

void PredictiveJointController::commandDriver(double voltage, DriverMode mode) 
{
	cDriverCommanded = true;
	nTargetVoltage = voltage;
	if (mode == DriverMode::TMVoltage)
	{
		nAppliedVoltage = voltageConverter->nextVoltage(voltage);
		nVoltage = voltageConverter->getAverageVoltage();
	}
	else if (mode == DriverMode::ConstantVoltage)
	{
		nAppliedVoltage = DRV8830::getNearestVoltage(voltage);
		voltageConverter->setActualVoltage(nAppliedVoltage);
		nVoltage = voltageConverter->getAverageVoltage();
	}
	else if (mode == DriverMode::Brake)
	{
		nVoltage = nAppliedVoltage = 0;
		voltageConverter->reset();
	}
	else if (mode == DriverMode::Coast)
	{
		nVoltage = nAppliedVoltage = 0;
		voltageConverter->reset();
	}

	nDriverMode = mode;
}

void PredictiveJointController::commitCommands()
{
	try
	{
		bus->selectAddress(servoModel->driverAddress);
		//This shouldn't happen. Shutdown motor to avoid damage due to unexpected condition;
		if (!cDriverCommanded)
		{
			emergencyHalt("Command wasn't applied during operation. Motor shutdown.");
		}
		else
		{
			if (nDriverMode == DriverMode::Brake)
				DRV8830::writeVoltageMode(bus, 0, DriveMode::BRAKE);
			else if (nDriverMode == DriverMode::ConstantVoltage || nDriverMode == DriverMode::TMVoltage)
				DRV8830::writeVoltage(bus, nAppliedVoltage);
			else // if (nDriverMode == DriverMode::Coast)
				DRV8830::writeVoltageMode(bus, 0, DriveMode::OFF);
		}
	}
	catch (std::runtime_error & e)
	{
		emergencyHalt("Cannot commit command");
		cout << "Error occured during commit: " << e.what();
	}
}


void PredictiveJointController::logState()
{
	csvLog 		
		<< cTime			<< Configuration::CsvSeparator
		<< jointStatus		<< Configuration::CsvSeparator
		<< cRawSensorAngle	<< Configuration::CsvSeparator
		<< cSensorAngle		<< Configuration::CsvSeparator
		<< cTargetAngle		<< Configuration::CsvSeparator
		<< cVelocity		<< Configuration::CsvSeparator
		<< cVelocityApproximationError << Configuration::CsvSeparator
		<< cTargetVelocity	<< Configuration::CsvSeparator
		<< speedControlStableTorque << Configuration::CsvSeparator
		//<< cDisturbanceTorque << Configuration::CsvSeparator
		<< cMotorTorque		<< Configuration::CsvSeparator
		<< cVoltage			<< Configuration::CsvSeparator
		<< nTargetVoltage	<< Configuration::CsvSeparator
		<< cAppliedVoltage	<< Configuration::CsvSeparator
		<< controlMode		<< Configuration::CsvSeparator;

		switch (controlMode) {
		case ControlMode::PositionControl:
			csvLog << positionControlState;
			break;
		case ControlMode::StepControl:
			csvLog << steppingState;
			break;
		case ControlMode::SpeedControl:
			csvLog << speedControlState;
			break;
		default:
			csvLog << 0;
		}

		csvLog << endl;

}

void PredictiveJointController::printState()
{
	cout 
		<< jointModel->name << ":  " 
		<< "RawSensor=" << AS5048::stepsToDegrees(cRawSensorAngle) << "deg, "
		<< "Sensor=" << AS5048::stepsToDegrees(cSensorAngle) << "deg, "
		<< "TargetAngle=" << AS5048::stepsToDegrees(cTargetAngle) << "deg, "
		<< "Velocity=" << AS5048::stepsToDegrees(cVelocity) << "deg/s, "
		<< "TargetVelocity=" << AS5048::stepsToDegrees(cTargetVelocity) << "deg/s"  
		<< endl
		<< "DisturbanceTorque=" << cDisturbanceTorque << "Nm, " 
		<< "MotorTorque=" << cMotorTorque << "Nm, "
		<< "MotorVoltage=" << cVoltage << " V, "
		<< "ControlMode=" << controlMode 
		<< endl;

	switch (controlMode) {
	case ControlMode::PositionControl:
		cout << "PosContMode=" << positionControlState << ", " << endl;
		break;
	case ControlMode::StepControl:
		cout 
			<< "SteppingState=" << steppingState << ", "
			<< "StepStartPosition=" << stepStartPosition << ", " 
			<< "StepVoltages=[ ";
		for (auto it=stepVoltages.begin(); it != stepVoltages.end(); it++) 
		{
			cout << (*it) << " ";
		}
		cout << "]" << endl;
		break;
	case ControlMode::SpeedControl:
	default:
		break;
	}

}


double PredictiveJointController::getMaxJointVelocity()
{
	const double AverageTorque = 0.5;

	return servoModel->getSpeedForTorqueVoltage(AverageTorque,servoModel->maxDriverVoltage);
}


JointModel * PredictiveJointController::getJointModel()
{
	return jointModel;
}


double PredictiveJointController::getCurrentAngle()
{
	return cSensorAngle;
}












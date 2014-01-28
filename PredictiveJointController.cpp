#include "PredictiveJointController.hpp"


using namespace std;

void PredictiveJointController::init()
{
	const double TorqueFilterTimeConstant = 0.05;
	const double PositionMovingAverageWindowSize = 4;
	const double SpeedMovingAverageWindowSize = 2;
	const double SpeedFilterTimeConstant = 0.05;
	
	filter_lowpass_for_motorTorque =	new LowpassFilter(TorqueFilterTimeConstant);
	filter_sma_angle_for_position =		new SimpleMovingAverage(PositionMovingAverageWindowSize);
	filter_sma_angle_for_speed =		new SimpleMovingAverage(SpeedMovingAverageWindowSize);
	filter_lowpass_speed =				new LowpassFilter(SpeedFilterTimeConstant);

	voltageConverter = new TimeMultiplexedVoltageConverter(2);

	controlMode = ControlMode::Disabled;
	jointStatus = JointStatus::New;
	motionPlan = NULL;

	if (Configuration::CsvLoggingEnabled)
	{
		csvLog.open("Data_" + jointModel->name + ".csv");
		csvLog <<
			"Time"				<< Configuration::CsvSeparator <<
			"RawSensorAngle"	<< Configuration::CsvSeparator <<
			"SensorAngle"		<< Configuration::CsvSeparator <<
			"TargetAngle"		<< Configuration::CsvSeparator <<
			"Velocity"			<< Configuration::CsvSeparator <<
			"TargetVelocity"	<< Configuration::CsvSeparator <<
			"DisturbanceTorque" << Configuration::CsvSeparator <<
			"MotorTorque"		<< Configuration::CsvSeparator <<
			"EffectiveVoltage"	<< Configuration::CsvSeparator <<
			"TargetVoltage"		<< Configuration::CsvSeparator <<
			"AppliedVoltage"	<< Configuration::CsvSeparator <<
			"ControlMode"		<< endl;
	}
	
	nVoltage = 0;
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
	validateMotionPlan(requestedMotionPlan);
	
	if (this->motionPlan != NULL)
		motionPlan.reset();		
	
	this->motionPlan = requestedMotionPlan;
	controlMode = ControlMode::SpeedControl;
	
}

void PredictiveJointController::validateMotionPlan(std::shared_ptr<JointMotionPlan> requestedMotionPlan)
{
	if (jointStatus != JointStatus::Active)
		throw std::runtime_error("Joint must be in Active state to execute a motion plan");
	
	if (requestedMotionPlan->finalAngle < jointModel->minAngle && requestedMotionPlan->finalAngle > jointModel->maxAngle)
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
	cVoltage = nVoltage;
	cAppliedVoltage = nAppliedVoltage;
	cTargetVoltage = nTargetVoltage;
	cDriverMode = nDriverMode;
	
	cDriverCommanded = false;	
		
	cNonZeroOffsetSensorAngle = getSensorAngleRegisterValue();
	cRawSensorAngle = MathUtil::offsetAngleSteps(cNonZeroOffsetSensorAngle,jointModel->sensorZeroPosition);
	cTime = timeSince(startTime);
	
	cVelocity = computeSpeed(cRawSensorAngle);	
	cSensorAngle = filterAngle(cRawSensorAngle); //Filter for position

	if (controlMode == ControlMode::Hold)
	{
		if (motionPlan == NULL)
			cTargetAngleDistance = AS5048::getAngleError(cSensorAngle,setpointHoldAngle);
		else
			cTargetAngleDistance = AS5048::getAngleError(cSensorAngle,motionPlan->finalAngle);

		cTargetVelocity = 0;
	}
	else
	{
		cTargetAngleDistance = AS5048::getAngleError(cSensorAngle,motionPlan->finalAngle);
		double targetDirection = MathUtils::sgn<double>(cTargetAngleDistance);
		cTargetVelocity = std::abs(motionPlan->getSpeedAtTime(MathUtil::timeSince(planStartTime)))*targetDirection;
	}
							   
	//Static torque resulting from current arm pose. Zero for now.
	//TODO: Also need to consider torque that results from angular acceleration
	cModelJointTorque = 0;
			
	//Torque experienced by the motor.
	cMotorTorque = servoModel->getTorqueForVoltageSpeed(cVoltage, cVelocity);	

	cDisturbanceTorque = filter_lowpass_for_motorTorque->next(cMotorTorque - cModelJointTorque);
}


/*
 Determining speed is non-trivial due to sensor noise.
 Sensor noise can be removed with a low-pass filter, but this may not be sufficient as the frequency
 of the noise is close to the frequency of the actual velocity changes.
 
 More investigation is needed to determine the optimal filtering strategy.
 Minimizing delay is critical to obtain smooth motion.
 */
double PredictiveJointController::computeSpeed(int rawSensorAngle)
{
	//	filter_lowpass_angle_for_speed
	//	filter_sma_angle_for_speed
	
	filter_sma_angle_for_speed->add(rawSensorAngle);
	double filteredAngle = filter_sma_angle_for_speed->avg();
	
	double delta = MathUtil::subtractAngles(filteredAngle,lFilteredAngleForSpeed,MathUtil::PI_STEPS);
	
	//lowpass filter on speed
	delta = filter_lowpass_speed->next(delta);
	
	return delta;
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
	const double MinimumStopTime = 0; //seconds
	const double VelocityZeroThreshold = 0.1;
	const double MinTorqueScale = 1.2;
	const double MinContinuousApproachDistance = 100;
	const double MinSpeedControlDistance = 200;
	const double MinApproachSpeed = 10; //steps/second
	const double SteppingModeReadDelay = 0.05; //seconds
	const double MaxSetpointError = 5; //steps
	const double MaxMotionForValidStep = 5;//steps
	const double BaseStepVoltage = 0.56; //V
	const double MinDisturbanceError = 100;	
	double MaximumStopTime = samplePeriod*1.5;
		
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
		
		if (std::abs(cTargetAngleDistance) < MinSpeedControlDistance)
		{
			controlMode = ControlMode::PositionControl;
			positionControlState = PositionControlState::Stabilizing;
		}
		else 
		{
			//Todo: accelerations		
			double expectedTorque = cDisturbanceTorque;
			commandDriver(servoModel->getVoltageForTorqueSpeed(expectedTorque,cTargetVelocity),DriverMode::TMVoltage);
		}

	}
	
	if (!cDriverCommanded && controlMode == ControlMode::PositionControl) {
		
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
				else if (stopIn < MaximumStopTime)
				{
					positionControlState = PositionControlState::Missed;
					commandDriver(0,DriverMode::Brake);
				}
				else 
				{
					//Keep steady and wait
					if (positionControlState == PositionControlState::Stabilizing) 
					{
						double minVoltage = servoModel->getVoltageForTorqueSpeed(cMotorTorque*MinTorqueScale,0);
						positionControlState = PositionControlState::Approaching;	
						commandDriver(minVoltage,DriverMode::ConstantVoltage);
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
				}
				else
				{
					positionControlState = PositionControlState::Missed;
				}
			}	
		}	

		//Could transition states in above block without generating a command
		if (!cDriverCommanded && positionControlState == PositionControlState::Missed) 
		{
			double targetDistance = MathUtil::subtractAngles(cTargetAngle,cSensorAngle,MathUtil::PI_STEPS);

			if (std::abs(targetDistance) < MinContinuousApproachDistance)
			{
				controlMode = ControlMode::StepControl;
				executeStep(BaseStepVoltage*MathUtils::sgn<double>(targetDistance));
			}
			else
			{
				double minVoltage = servoModel->getVoltageForTorqueSpeed(cMotorTorque*MinTorqueScale,MathUtils::sgn<double>(targetDistance)*MinApproachSpeed);
				positionControlState = PositionControlState::Approaching;										
				commandDriver(minVoltage,DriverMode::ConstantVoltage);
			}
		}
	}
	
	if (!cDriverCommanded && controlMode == ControlMode::StepControl) {
		
		//Filter?
		double targetDistance = MathUtil::subtractAngles(cTargetAngle,cSensorAngle,MathUtil::PI_STEPS);

		if (std::abs(targetDistance) > MaxSetpointError) {

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
				commandDriver(0,DriverMode::Brake);
				break;
			case SteppingState::Reading:
				if (MathUtil::timeSince(readDelayStart) > SteppingModeReadDelay) 
				{
					int stepDirection = -MathUtils::sgn<double>(targetDistance);
					
					//Overshoot, reverse direction and step again.
					if (MathUtils::sgn<double>(targetDistance) != MathUtils::sgn<double>(stepInitialTargetDistance))
					{						
						executeStep(BaseStepVoltage*stepDirection);
					}
					//Didn't go far enough to count as motion, or went backwards. Step again with higher voltage
					else if ((targetDistance - stepInitialTargetDistance) < MaxMotionForValidStep*stepExpectedDirection) 
					{
						int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltages.at(0))) + 1;
						executeStep(vSteps*stepDirection);
					}
					//Didn't make it, repeat step at same voltage
					else 
					{
						int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltages.at(0)));
						executeStep(DRV8830::stepsToVoltage(vSteps)*stepDirection);
					}
				}
				break;
			}
		}
		else {
			controlMode = ControlMode::Hold;
		}
	}

	commitCommands();
	logState();
}

void PredictiveJointController::executeStep(double voltage) 
{
	stepInitialTargetDistance = MathUtil::subtractAngles(cTargetAngle,cSensorAngle,MathUtil::PI_STEPS);

	stepVoltages.clear();
	stepVoltages.push_back(voltage);
	stepVoltages.push_back(0);

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
		<< cRawSensorAngle	<< Configuration::CsvSeparator
		<< cSensorAngle		<< Configuration::CsvSeparator
		<< cTargetAngle		<< Configuration::CsvSeparator
		<< cVelocity		<< Configuration::CsvSeparator
		<< cTargetVelocity	<< Configuration::CsvSeparator
		<< cDisturbanceTorque << Configuration::CsvSeparator
		<< cMotorTorque		<< Configuration::CsvSeparator
		<< cVoltage			<< Configuration::CsvSeparator
		<< cTargetVoltage	<< Configuration::CsvSeparator
		<< cAppliedVoltage	<< Configuration::CsvSeparator
		<< controlMode		<< Configuration::CsvSeparator
		<< endl;
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












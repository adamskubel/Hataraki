#include "PredictiveJointController.hpp"



double timeSince(timespec & sinceTime){
	timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	return MathUtil::getTimeDelta(sinceTime,now);
}

void PredictiveJointController::setCurrentState()
{		
	cVoltage = nVoltage;
	cDriverMode = nDriverMode;
	cDriverCommanded = false;	

	cRawSensorAngle = MathUtil::offsetAngleSteps(AS5048::getSensorAngleSteps(bus),servoModel->sensorZeroPosition);
	cTime = timeSince(startTime);
	
	cVelocity = computeSpeed(cRawSensorAngle);	
	cSensorAngle = filterAngle(cRawSensorAngle); //Filter for position
	
	cTargetAngleDistance = AS5048::getAngleError(cSensorAngle,motionPlan->finalAngle);
	cTargetVelocity = motionPlan->getSpeedAtTime(cTime);

	//Static torque resulting from current arm pose
	double staticJointTorque = 0; //PoseDynamics::getInstance().computeTorqueOnJoint(0,0); TODO: precomputing torques during sleep time	
	//Also need to consider torque that results from angular acceleration	
	cJointTorque = staticJointTorque;
			
	cMotorTorque = servoModel->getTorqueForVoltageSpeed(cVoltage, cVelocity);	
}



//int PredictiveJointController::getControlMode()
//{
//	//Close to setpoint, use position control
//	//Minimum velocity determined by torque/speed/voltage function
//
//	double minVoltage = servoModel->getVoltageForTorqueSpeed(cJointTorque,0);
//	double minVelocity = servoModel->getSpeedForTorqueVoltage(cJointTorque,DRV8830::getNearestVoltage(minVoltage));
//
//	if (MathUtil::abs(cTargetAngleDistance) < minVelocity*10*sampleTime)
//	{
//		return ControlMode::PositionControl;
//	}
//
//	return ControlMode::SpeedControl;
//}

void PredictiveJointController::run()
{
	if (!active) return;

	const double MinimumStopTime = 0.1; //seconds 
	const double VelocityZeroThreshold = 0.1;
	const double MinTorqueScale = 1.2;
	const double MinContinuousApproachDistance = 100;
	const double MinSpeedControlDistance = 200;
	const double MinApproachSpeed = 0.5; //LOL 
	const double SteppingModeReadDelay = 0.05; //seconds
	const double MaxSetpointError = 5; //steps
	const double MaxMotionForValidStep = 5;//steps
	const double BaseStepVoltage = 0.56; //V

	setCurrentState();		

	performSafetyChecks();

	
	if (controlMode == ControlMode::SpeedControl) {
		
		if (std::abs(cTargetAngleDistance) < MinSpeedControlDistance)
		{
			controlMode = ControlMode::PositionControl;
			positionControlState = PositionControlState::Stabilizing;
		}
		else 
		{
			//Todo: accelerations		
			double torqueError =  cMotorTorque - cJointTorque;
			double expectedTorque = cJointTorque + torqueFilter->next(torqueError); //Should filtering be done here? Speed is already filtered.

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
			//If not moving, then transition to stepping mode
			else 
			{
				positionControlState = PositionControlState::Missed;
			}	
		}	

		//Could transition states in above block without generating a command
		if (!cDriverCommanded && positionControlState == PositionControlState::Missed) 
		{
			double targetDistance = MathUtil::subtractAngles(cTargetAngle,cSensorAngle,MathUtil::PI_STEPS);

			if (std::abs(targetDistance) < MinContinuousApproachDistance)
			{
				controlMode = ControlMode::StepControl;
				executeStep(BaseStepVoltage*stepDirection);
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
				nextVoltage = stepVoltages.at(stepVoltageIndex++);
				if (stepVoltageIndex >= stepVoltages.size())
					steppingState = SteppingState::Braking;
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


		}
	}

	//This shouldn't happen. Shutdown motor to avoid damage.
	if (!cDriverCommanded)
	{
		commandDriver(0,DriverMode::Coast);
		cout << "Error! Command wasn't applied during operation. Motor shutdown." << endl;
	}
}

void PredictiveJointController::executeStep(double voltage) 
{
	stepInitialTargetDistance = MathUtil::subtractAngles(cTargetAngle,cSensorAngle,MathUtil::PI_STEPS);

	stepVoltages.clear();
	stepVoltages.push_back(voltage);
	stepVoltages.push_back(0);

	stepStartPosition = cSensorAngle;
	stepExpectedDirection = MathUtils::sgn<double>(voltage); //Assuming direction is the same sign as voltage
	
	steppingState = SteppingState::Energizing;

	stepVoltageIndex = 0;
	commandDriver(stepVoltages.at(stepVoltageIndex++),DriverMode::ConstantVoltage);
}

void PredictiveJointController::commandDriver(double voltage, DriverMode mode) 
{
	cDriverCommanded = true;

	double commandVoltage;
	if (mode == DriverMode::TMVoltage)
	{
		commandVoltage = voltageConverter->nextVoltage(voltage);
		nVoltage = voltageConverter->getAverageVoltage();
	}
	else if (mode == DriverMode::ConstantVoltage)
	{
		commandVoltage = DRV8830::getNearestVoltage(voltage);
		voltageConverter->setActualVoltage(commandVoltage);
		nVoltage = voltageConverter->getAverageVoltage();
	}
	else if (mode == DriverMode::Brake)
	{
		nVoltage = commandVoltage = 0;
		voltageConverter->reset();
	}	
	else if (mode == DriverMode::Coast)
	{
		nVoltage = commandVoltage = 0;
		voltageConverter->reset();
	}
}

//double PredictiveJointController::getVoltageForStep(double voltageLevel)
//{
//	const double defaultStepVoltage = 0.7;
//	return MathUtils::sgn<double>(vol)*DRV8830::getNearestVoltage(defaultStepVoltage);
//}




/*
 Determining speed is non-trivial due to sensor error, and primarily, sensor noise.
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





















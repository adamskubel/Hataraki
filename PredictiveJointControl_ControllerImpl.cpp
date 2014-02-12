#include "PredictiveJointController.hpp"

namespace ControllerConfiguration 
{
	const double MinimumStopTime = 0; //seconds
	const double VelocityZeroThreshold = 30;
	const double MinTorqueScale = 1.2;
	const double MinContinuousApproachDistance = 30;
	
	const double BaseApproachVoltage = 0.68;
	const double MaximumStopTimeScale = 4.0;
	const double MinApproachSpeed = 400; //steps/second

	const double SteppingModeReadDelay = 0.03; //seconds
	const double MaxMotionForValidStep = 5;//steps
	const double LostMotionFlipVoltage = 0.48;// 1.04; //V

	//Position Hold
	const double MaxSetpointError = 5; //steps
	const double MinDisturbanceError = 15;	
	const double MinFlipTriggerVelocity = 500;
	
	//Speed Control
	const double MinSpeedControlMeasureDelay = 0.03; //seconds
	const double MaxSpeedControlMeasureDelay = 0.09; //seconds
	const double MinSpeedControlDistance = 200;
	const double MinVelocityRValue = 0.95;
	const double BaseSpeedControlMeasureTorque = 0.01;
	const double MaxVelocitySetpointError = 20; //steps/second
	const double MinNonZeroVelocity = 50; //steps/second
	const double MaxPossibleZeroVelocity = 150;
	const double MinAverageOffsetForNonZeroVelocity = 4;


	const double MinimumPredictedTorque = 0.01; //N*m

	const double BaseStepSpeed = 800.0;// steps/second

	const bool GravityFlipTriggerEnabled = false;
};

using namespace ControllerConfiguration;
using namespace std;

void PredictiveJointController::run()
{
	timespec start;
	TimeUtil::setNow(start);

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
	
	setTargetState();
	
	if (jointStatus == JointStatus::Error) return;
		
	handleUserRequests();

	if (!cDriverCommanded && controlMode == ControlMode::External)
	{
		runExternalController();

		if (!cDriverCommanded)
		{
			emergencyHalt("External controller failed to send a command during execution.");
			return;
		}
	}

	if (!cDriverCommanded && controlMode == ControlMode::Hold) 
	{
		doPositionHoldControl();
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

	TimeUtil::assertTime(start,jointModel->name + ".run()");
}

void PredictiveJointController::runExternalController()
{
	externalController();
}



void PredictiveJointController::performSafetyChecks()
{
	if (cRawSensorAngle < jointModel->minAngle || cRawSensorAngle > jointModel->maxAngle || 
		cSensorAngle < jointModel->minAngle || cSensorAngle > jointModel->maxAngle)
	{
		stringstream ss;
		ss << "Exceeded angle limits. Angle = " << AS5048::stepsToDegrees(cRawSensorAngle) << " [" << AS5048::stepsToDegrees(jointModel->minAngle) << "," << AS5048::stepsToDegrees(jointModel->maxAngle) << "]" << endl;
		emergencyHalt(ss.str());
	}
}

void PredictiveJointController::emergencyHalt(std::string reason)
{
	try
	{
		controlMode = ControlMode::Disabled;
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

void PredictiveJointController::doPositionHoldControl()
{
	bool commanded = false;

	if (servoModel->controllerConfig.externalDisturbanceFlipTriggerEnabled && std::abs(cTargetAngleDistance) > MaxSetpointError)
	{
		//Moving away from rotational stop
		if ((int)MathUtils::sgn<double>(cTargetAngleDistance) == -expectedRotationalStopDirection)
		{
			//double averageVelocity = getAverageSpeedForWindow(3);

			//if ((int)MathUtils::sgn<double>(averageVelocity) == expectedRotationalStopDirection && std::abs(averageVelocity) > MinFlipTriggerVelocity)
			{
				vector<double> flipPattern;
				double direction = MathUtils::sgn<double>(cTargetAngleDistance);

				for (auto it=servoModel->controllerConfig.externalDisturbanceFlipVoltagePattern.begin(); it!= servoModel->controllerConfig.externalDisturbanceFlipVoltagePattern.end();it++)
				{
					flipPattern.push_back((*it) * direction);
				}
							
				controlMode = ControlMode::StepControl;
				executeStep(flipPattern);	
				commanded = true;
			}
		}		
	}
	
	if (!commanded)
	{
		if (std::abs(cTargetAngleDistance) > MinDisturbanceError)
		{
			controlMode = ControlMode::PositionControl;
			positionControlState = PositionControlState::Missed;
		}
		else 
		{
			readyForCommand = true;
			commandDriver(0,DriverMode::Brake);
		}
	}
}

//Feedback based controller
void PredictiveJointController::doSpeedControl()
{
	if (std::abs(cTargetAngleDistance) < MinSpeedControlDistance)	
	{
		controlMode = ControlMode::PositionControl;
		positionControlState = PositionControlState::Stabilizing;
		isTorqueEstimateValid = false;
		stableTorqueEstimate = cPredictedTorque;
	}
	else 
	{
		double cVelocityError = cTargetVelocity - cVelocity;
		velocityErrorIntegral += cVelocityError * (cTime - lTime);

		if (speedControlState == SpeedControlState::Measuring)
		{
			if (MathUtil::timeSince(speedControlMeasureStart) > MinSpeedControlMeasureDelay && cVelocityApproximationError < MinVelocityRValue)
			{
				speedControlState = SpeedControlState::Adjusting;
			}
			else if (MathUtil::timeSince(speedControlMeasureStart) > MaxSpeedControlMeasureDelay)
			{
				speedControlState = SpeedControlState::Adjusting;
			}
			else
			{
				commandDriver(servoModel->getVoltageForTorqueSpeed(cControlTorque,cTargetVelocity),DriverMode::TMVoltage);	
			}
		}

		if (!cDriverCommanded && speedControlState == SpeedControlState::Adjusting)
		{
			cControlTorque += ((cVelocityError * speedControlProportionalGain) + (velocityErrorIntegral * speedControlIntegralGain))/-servoModel->getTorqueSpeedSlope();
			commandDriver(servoModel->getVoltageForTorqueSpeed(cControlTorque,cTargetVelocity),DriverMode::TMVoltage);	
			speedControlState = SpeedControlState::Measuring;
			MathUtil::setNow(speedControlMeasureStart);
		}

		velocityErrorIntegral *= 0.95;
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
			double predictedDistance = cTargetAngle - predictedAngle;//MathUtil::subtractAngles(cTargetAngle, predictedAngle, MathUtil::PI_STEPS);
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
					double approachVoltage =  servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,cTargetVelocity); 
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
			double stepVoltage = servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,MathUtils::sgn<double>(cTargetAngleDistance)*BaseStepSpeed);
			executeStep(stepVoltage,1,1);
			isTorqueEstimateValid = false;
		}
		else
		{
			//double approachVoltage = BaseApproachVoltage*MathUtils::sgn<double>(cTargetAngleDistance);
			double approachVoltage = servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,MathUtils::sgn<double>(cTargetAngleDistance)*MinApproachSpeed);
			positionControlState = PositionControlState::Approaching;										
			commandDriver(approachVoltage,DriverMode::ConstantVoltage);
			isTorqueEstimateValid = false;
		}
	}
}

void PredictiveJointController::doStepControl()
{	
	switch (steppingState)
	{
	case SteppingState::Energizing:
		if (stepVoltageIndex < stepVoltages.size())			
		{
			commandDriver(stepVoltages.at(stepVoltageIndex++),DriverMode::ConstantVoltage);			
				
			if (stepVoltageIndex >= stepVoltages.size())			
				steppingState = SteppingState::Braking;
		}
		else				
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
			int stepDirection = MathUtils::sgn<double>(cTargetAngleDistance);

			//Reached target, go to hold mode
			if (std::abs(cTargetAngleDistance) <= MaxSetpointError) 
			{
				controlMode = ControlMode::Hold;
				commandDriver(0,DriverMode::Brake);	
			}
			//Too far away for stepping mode, go to position mode
			else if (std::abs(cTargetAngleDistance) > MinContinuousApproachDistance)
			{
				double approachVoltage = servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,MathUtils::sgn<double>(cTargetAngleDistance)*MinApproachSpeed);
				positionControlState = PositionControlState::Approaching;										
				commandDriver(approachVoltage,DriverMode::ConstantVoltage);
				isTorqueEstimateValid = false;
			}
			//Overshoot, reverse direction and step again.
			else if (MathUtils::sgn<double>(cTargetAngleDistance) != MathUtils::sgn<double>(stepInitialTargetDistance))
			{						
				double stepVoltage = servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,stepDirection*BaseStepSpeed);
				executeStep(stepVoltage,1,1);
			}
			//Didn't go far enough to count as motion, or went backwards. Step again with higher voltage
			else if (std::abs(cTargetAngleDistance - stepInitialTargetDistance) < MaxMotionForValidStep) 
			{					
				int length = 1;
				int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltageIntegral));
				if (std::abs(vSteps) >= getMaxVoltageSteps())
					length = 2;
				else
					vSteps++;

				executeStep(DRV8830::stepsToVoltage(vSteps)*stepDirection,length,1);
			}
			//Didn't make it, repeat step at same voltage
			else 
			{
				int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltageIntegral));
				executeStep(DRV8830::stepsToVoltage(vSteps)*stepDirection,1,1);
			}
		}
		else
		{
			commandDriver(0,DriverMode::Brake);
		}
		break;
	}
	
}


void PredictiveJointController::executeStep(double voltage, int energizeLength, int coastStepCount) 
{
	stepInitialTargetDistance = cTargetAngleDistance; 

	stepVoltages.clear();
	
	for (int i=0;i<energizeLength;i++)
		stepVoltages.push_back(voltage);

	for (int i=0;i<coastStepCount;i++)
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


void PredictiveJointController::executeStep(vector<double> & voltagePattern) 
{
	stepVoltages = voltagePattern;
	
	stepVoltageIntegral = std::accumulate(stepVoltages.begin(), stepVoltages.end(), 0.0);
	stepExpectedDirection = MathUtils::sgn<double>(stepVoltageIntegral); 
	stepInitialTargetDistance = cTargetAngleDistance; 
	stepStartPosition = cSensorAngle;

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
	if (voltage > servoModel->maxDriverVoltage)
		voltage = servoModel->maxDriverVoltage;
	if (voltage < -servoModel->maxDriverVoltage)
		voltage = -servoModel->maxDriverVoltage;

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
		//This shouldn't happen. Shutdown motor to avoid damage due to unexpected condition;
		if (!cDriverCommanded)
		{
			emergencyHalt("Command wasn't applied during operation. Motor shutdown.");
		}
		else
		{
			int driverCommand;
			if (nDriverMode == DriverMode::Brake)
				driverCommand = DRV8830::buildCommand(0,DriveMode::BRAKE);	
			else if (nDriverMode == DriverMode::ConstantVoltage || nDriverMode == DriverMode::TMVoltage)
				driverCommand = DRV8830::buildCommand(nAppliedVoltage);
			else if (nDriverMode == DriverMode::Coast)
				driverCommand = DRV8830::buildCommand(0, DriveMode::OFF);
			else
			{
				emergencyHalt("Invalid driver mode.");
				return;
			}
			
			timespec startTime;
			MathUtil::setNow(startTime);
			if (cDriverCommand != driverCommand)
			{
				bus->selectAddress(servoModel->driverAddress);
				DRV8830::writeCommand(bus,driverCommand);
				cDriverCommand = driverCommand;
			}
			cDriverWriteTime = MathUtil::timeSince(startTime)*1000.0;
		}
	}
	catch (std::runtime_error & e)
	{
		emergencyHalt("Cannot commit command");
		cout << "Error occured during commit: " << e.what();
	}
}

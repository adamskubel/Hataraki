#include "PredictiveJointController.hpp"

namespace ControllerConfiguration 
{
	const double MinimumStopTime = 0; //seconds
	const double VelocityZeroThreshold = 30;
	const double MinTorqueScale = 1.2;
	
	const double MaximumStopTime = 0.03;

	const double SteppingModeReadDelay = 0.03; //seconds
	const double MaxMotionForValidStep = 5;//steps

	//Position Hold
	const double MinDisturbanceError = 15;	
	const double MinFlipTriggerVelocity = 500;
	
	//Speed Control
	const double MinSpeedControlMeasureDelay = 0.05; //seconds
	const double MaxSpeedControlMeasureDelay = 0.10; //seconds
	const double MinVelocityRValue = 0.95;
	const double MinNonZeroVelocity = 50; //steps/second

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
	
	if (haltRequested)
	{
		emergencyHalt("Joint running with control disabled");
		return;
	}

	setCurrentState();
	performSafetyChecks();		
	if (jointStatus == JointStatus::Error) return;
		
	setTargetState();
	setCurrentTorqueStates();

	if (dynamicControl)
	{
		doDynamicControl();
	}
	else
	{
		doStaticControl();
	}

	commitCommands();

	if (Configuration::CsvLoggingEnabled) logState();
	
	TimeUtil::assertTime(start,jointModel->name + ".run()");
}

void PredictiveJointController::runExternalController()
{
	externalController();
}


void PredictiveJointController::setTargetState()
{
	if (motionPlan == NULL)
	{
		emergencyHalt("null motion plan");
		return;
	}

	if (motionPlanComplete)
	{	
		cTargetAngle = motionPlan->finalAngle;
		dynamicControl = false;
	}
	else
	{		
		double remainingTime = TimeUtil::timeUntil(motionPlan->endTime);	
		bool isMoving = std::abs(cVelocity) > 100.0;

		dynamicControl = isMoving || remainingTime > -0.1;
				
		if (dynamicControl)
		{
			double t = TimeUtil::timeSince(motionPlan->startTime);
			cTargetAngle = motionPlan->x(t);
			cTargetVelocity = motionPlan->dx(t);
			cTargetAcceleration = motionPlan->ddx(t);
			cDynamicTorque = 0;// PoseDynamics::getInstance().getTorqueForAcceleration(jointModel->index,AS5048::stepsToRadians(cTargetAcceleration));
		}
		else
		{
			motionPlanComplete = true;
			cTargetAngle = motionPlan->finalAngle;
		}
	}

	cPlanTargetVelocity = cTargetVelocity;
	cTargetAngleDistance = cTargetAngle - cSensorAngle;
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
		haltRequested = true;
		bus->selectAddress(servoModel->driverAddress);
		DRV8830::writeVoltageMode(bus, 0, DriveMode::OFF);		
		haltRequested = false;
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

	//if (servoModel->controllerConfig.externalDisturbanceFlipTriggerEnabled && std::abs(cTargetAngleDistance) > MaxSetpointError)
	//{
	//	//Moving away from rotational stop
	//	if ((int)sgn(cTargetAngleDistance) == -expectedRotationalStopDirection)
	//	{
	//		//double averageVelocity = getAverageSpeedForWindow(3);

	//		//if ((int)sgn(averageVelocity) == expectedRotationalStopDirection && std::abs(averageVelocity) > MinFlipTriggerVelocity)
	//		{
	//			vector<double> flipPattern;
	//			double direction = sgn(cTargetAngleDistance);

	//			for (auto it=servoModel->controllerConfig.externalDisturbanceFlipVoltagePattern.begin(); it!= servoModel->controllerConfig.externalDisturbanceFlipVoltagePattern.end();it++)
	//			{
	//				flipPattern.push_back((*it) * direction);
	//			}
	//						
	//			controlMode = ControlMode::StepControl;
	//			executeStep(flipPattern);	
	//			commanded = true;
	//		}
	//	}		
	//}
	
	if (!commanded)
	{
		if (std::abs(cTargetAngleDistance) > MinDisturbanceError)
		{
			staticControlMode = StaticControlMode::Stepping;
			doStepControl(cTargetAngle);
		}	
		else
		{
			readyForCommand = true;
			commandDriver(0,DriverMode::Brake);
		}
	}
}

double PredictiveJointController::estimateTimeToPosition(double position)
{
	double predictedAngle = cSensorAngle + (cVelocity*servoModel->sensorDelay); 
	double predictedDistance = position - predictedAngle;
	return predictedDistance / cVelocity;
}

void PredictiveJointController::doDynamicControl()
{		
	
	if (dynamicControlMode == DynamicControlMode::Starting)
	{
		//if (!config->motionStartEnabled || startModeIndex >= config->startVoltagePattern.size())
		dynamicControlMode = DynamicControlMode::Travelling;
		//else
		//{
		//	commandDriver(config->startVoltagePattern.at(startModeIndex)*sgn(cPlanTargetVelocity),DriverMode::ConstantVoltage);
		//	startModeIndex++;
		//}
	}
	
	if (dynamicControlMode == DynamicControlMode::Travelling)
	{
		double dynamicAngleError = cTargetAngle - cSensorAngle;
		double errorDerivative = dynamicAngleError - lDynamicPositionError;
		lDynamicPositionError = dynamicAngleError;

		double velocityCorrection = dynamicAngleError * config->velocityCorrectionProportionalGain + errorDerivative * config->velocityCorrectionDerivativeGain;
		velocityCorrection /= samplePeriod; 

		cTargetVelocity = cPlanTargetVelocity+velocityCorrection;
		double finalAngleError = motionPlan->finalAngle - cSensorAngle;

		//Approaching, stick to plan velocity
		if (std::abs(finalAngleError) < config->approachDistanceThreshold)
		{			
			dynamicControlMode = DynamicControlMode::Approaching;
		}
	}
	
	if (dynamicControlMode == DynamicControlMode::Approaching)
	{
		cTargetVelocity = cPlanTargetVelocity;

		double stopIn = estimateTimeToPosition(motionPlan->finalAngle) - servoModel->driverDelay;
		if (stopIn < MaximumStopTime)
		{
			dynamicControlMode = DynamicControlMode::Stopping;
			commandDriver(0,DriverMode::Coast);
		}	
		else
		{
			doSpeedControl();
		}
	}	
	else if (dynamicControlMode == DynamicControlMode::Stopping)
	{
		commandDriver(0,DriverMode::Brake);
		motionPlanComplete = true;
	}
	else
	{
		doSpeedControl();
	}	
}


void PredictiveJointController::doStaticControl()
{	
	if (staticControlMode == StaticControlMode::Holding)
	{
		doPositionHoldControl();	
	}
	
	if (!cDriverCommanded && staticControlMode == StaticControlMode::Stepping)
	{
		if (doStepControl(cTargetAngle))
		{
			staticControlMode = StaticControlMode::Holding;
		}
	}
}

void PredictiveJointController::doSpeedControl()
{
	double expectedVelocity = servoModel->getSpeedForTorqueVoltage(cControlTorque,cVoltage);

	double cVelocityError = expectedVelocity - cVelocity;
	velocityErrorIntegral += cVelocityError * (cTime - lTime);
	
	double dVError = (cVelocityError - lVelocityError)/(cTime - lTime);

	if (speedControlState == SpeedControlState::Measuring)
	{
		if (TimeUtil::timeSince(speedControlMeasureStart) > MinSpeedControlMeasureDelay && cVelocityApproximationError < 0.7) //MinVelocityRValue)
		{
			speedControlState = SpeedControlState::Adjusting;
		}
		else if (TimeUtil::timeSince(speedControlMeasureStart) > MaxSpeedControlMeasureDelay)
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
		double velocityError = ((cVelocityError * speedControlProportionalGain) + (velocityErrorIntegral * speedControlIntegralGain) + (dVError * config->speedControlDerivativeGain));
		
		cControlTorque -= velocityError/servoModel->getTorqueSpeedSlope();
		if (sgn(cControlTorque) != sgn(cTargetVelocity))
			cControlTorque = 0;

		commandDriver(servoModel->getVoltageForTorqueSpeed(cControlTorque,cTargetVelocity),DriverMode::TMVoltage);	
		speedControlState = SpeedControlState::Measuring;
		TimeUtil::setNow(speedControlMeasureStart);
	}

	velocityErrorIntegral *= 0.95;
	lVelocityError = cVelocityError;
}


bool PredictiveJointController::doStepControl(double targetAngle)
{	
	double distanceToTarget = targetAngle - cSensorAngle;

	switch (steppingState)
	{
	//case SteppingState::New:
	//	double stepVoltage = servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,sgn(distanceToTarget)*BaseStepSpeed);
	//	executeStep(stepVoltage,1,1);
	//	isTorqueEstimateValid = false;
	//	break;
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
		TimeUtil::setNow(readDelayStart);
		commandDriver(0,DriverMode::Coast);
		break;
	case SteppingState::Reading:
		if (TimeUtil::timeSince(readDelayStart) > SteppingModeReadDelay) 
		{
			int stepDirection = sgn(distanceToTarget);

			//Reached target, brake and return true to indicate position has been reached
			if (std::abs(distanceToTarget) <= config->maxSetpointError) 
			{
				commandDriver(0,DriverMode::Brake);	
				return true;
			}
			//Overshoot, reverse direction and step again with lower voltage
			else if (sgn(cTargetAngleDistance) != sgn(stepInitialTargetDistance))
			{						
				//double stepVoltage = servoModel->getVoltageForTorqueSpeed(stableTorqueEstimate,stepDirection*BaseStepSpeed);
				//executeStep(stepVoltage,1,1);
				
				int length = 1;
				int vSteps = DRV8830::voltageToSteps(std::abs(stepVoltageIntegral));
				vSteps--;
				
				executeStep(DRV8830::stepsToVoltage(vSteps)*stepDirection,length,1);
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
	return false;
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
	stepExpectedDirection = sgn(voltage); //Assuming direction is the same sign as voltage
	
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
	stepExpectedDirection = sgn(stepVoltageIntegral); 
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
			TimeUtil::setNow(startTime);
			if (cDriverCommand != driverCommand)
			{
				bus->selectAddress(servoModel->driverAddress);
				DRV8830::writeCommand(bus,driverCommand);
				cDriverCommand = driverCommand;
			}
			cDriverWriteTime = TimeUtil::timeSince(startTime)*1000.0;
		}
	}
	catch (std::runtime_error & e)
	{
		emergencyHalt("Cannot commit command");
		cout << "Error occured during commit: " << e.what();
	}
}

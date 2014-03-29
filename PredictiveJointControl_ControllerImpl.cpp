#include "PredictiveJointController.hpp"

namespace ControllerConfiguration 
{
	const double MaximumStopTime = 0.03;
	const double SteppingModeReadDelay = 0.03; //seconds
	const double MaxMotionForValidStep = 5;//steps

	//Position Hold
	const double MinDisturbanceError = 15;
	
	//Speed Control
	const double MinSpeedControlMeasureDelay = 0.00; //seconds
	const double MaxSpeedControlMeasureDelay = 0.05; //seconds
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

	cRunTime = TimeUtil::timeSince(start)*1000.0;
	if (Configuration::CsvLoggingEnabled) logState();
	
	TimeUtil::assertTime(start,jointModel->name + ".run()");
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
			cTargetAcceleration = 0; // motionPlan->ddx(t);
			cDynamicTorque = 0; // PoseDynamics::getInstance().getTorqueForAcceleration(jointModel->index,AS5048::stepsToRadians(cTargetAcceleration));
			cPlanTargetVelocity = cTargetVelocity;
		}
		else
		{
			motionPlanComplete = true;
			cTargetAngle = motionPlan->finalAngle;
		}
	}

	cTargetAngleDistance = cTargetAngle - cSensorAngle;
}

void PredictiveJointController::performSafetyChecks()
{
	if (!jointModel->continuousRotation)
	{
		if ((cSensorAngle < jointModel->minAngle || cSensorAngle > jointModel->maxAngle) ||
			(cRawSensorAngle < jointModel->minAngle || cRawSensorAngle > jointModel->maxAngle))
		{
			stringstream ss;
			ss << jointModel->name << " exceeded angle limits. Angle = " << AS5048::stepsToDegrees(cSensorAngle) << " [" << AS5048::stepsToDegrees(jointModel->minAngle) << "," << AS5048::stepsToDegrees(jointModel->maxAngle) << "]" << endl;
			emergencyHalt(ss.str());
		}
	}
}

void PredictiveJointController::emergencyHalt(std::string reason)
{
	try
	{
		haltRequested = true;
		bus[servoModel->driverBus]->selectAddress(servoModel->driverAddress);
		DRV8830::writeVoltageMode(bus[servoModel->driverBus], 0, DriveMode::OFF);		
		haltRequested = false;
		jointStatus = JointStatus::Error;		
		cout << "Joint " << jointModel->name << " is now offline" << endl;
		cout << "Reason for shutdown: " << reason << endl;
		AsyncLogger::log("JointController", reason);
	}
	catch (std::runtime_error & e)
	{
		cout << "HALP I CANNOT STOPS I HAZ EXCPTIONZ " << e.what() << endl;
	}
}

void PredictiveJointController::doPositionHoldControl()
{
	if (config->stepControlEnabled && std::abs(cTargetAngleDistance) > MinDisturbanceError)
	{
		staticControlMode = StaticControlMode::Stepping;
		doStepControl();
	}	
	else
	{
		commandDriver(0,DriverMode::Brake);
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
		dynamicControlMode = DynamicControlMode::Travelling;
	}
	
	if (dynamicControlMode == DynamicControlMode::Travelling)
	{
		double dynamicAngleError = cTargetAngle - cSensorAngle;
		double errorDerivative = (dynamicAngleError - lDynamicPositionError)/(cTime - lTime);
		lDynamicPositionError = dynamicAngleError;

		double velocityCorrection = dynamicAngleError * config->velocityCorrectionProportionalGain + errorDerivative * config->velocityCorrectionDerivativeGain;
		
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
	if (!config->stepControlEnabled || staticControlMode == StaticControlMode::Holding)
	{
		doPositionHoldControl();	
	}
	
	if (!cDriverCommanded && staticControlMode == StaticControlMode::Stepping)
	{
		if (doStepControl())
		{
			staticControlMode = StaticControlMode::Holding;
		}
	}
}

void PredictiveJointController::doSpeedControl()
{
	double expectedVelocity;
	if (config->useTargetFeedback)
		expectedVelocity = servoModel->getSpeedForTorqueVoltage(cTargetVelocity,cVoltage);
	else
		expectedVelocity = servoModel->getSpeedForTorqueVoltage(cControlTorque,cVoltage);

	//Stall check
//	if (abs(expectedVelocity) > 1.0 && cVelocityApproximationError > 0.95 && abs(cVelocity) < 1.0)
//	{		
//		double voltageAdjust = 0.08 * sgn(cVoltage);
//		commandDriver(cVoltage+voltageAdjust,DriverMode::TMVoltage);	
//		speedControlState = SpeedControlState::Stalled;
//	}
//	else if (speedControlState == SpeedControlState::Stalled)
//	{
//		cControlTorque = servoModel->getTorqueForVoltageSpeed(cVoltage,cTargetVelocity);
//		speedControlState = SpeedControlState::Measuring;
//	}
//
//	if (speedControlState != SpeedControlState::Stalled)
	{
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
}


bool PredictiveJointController::doStepControl()
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
		TimeUtil::setNow(readDelayStart);
		commandDriver(0,DriverMode::Coast);
		break;
	case SteppingState::Reading:
		if (TimeUtil::timeSince(readDelayStart) > SteppingModeReadDelay) 
		{
			int stepDirection = sgn(cTargetAngleDistance);

			//Reached target, brake and return true to indicate position has been reached
			if (std::abs(cTargetAngleDistance) <= config->maxSetpointError)
			{
				commandDriver(0,DriverMode::Brake);	
				return true;
			}
			//Overshoot, reverse direction and step again with lower voltage
			else if (sgn(cTargetAngleDistance) != sgn(stepInitialTargetDistance))
			{										
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
		nAppliedVoltage = voltageConverter->nextVoltage((cTime-lTime),voltage);
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
				bus[servoModel->driverBus]->selectAddress(servoModel->driverAddress);
				DRV8830::writeCommand(bus[servoModel->driverBus],driverCommand);
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

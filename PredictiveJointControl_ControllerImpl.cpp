#include "PredictiveJointController.hpp"


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
		cPlanTargetVelocity = sgn(cTargetAngle - cSensorAngle);
		dynamicControl = false;
	}
	else
	{		
		double remainingTime = TimeUtil::timeUntil(motionPlan->endTime);	
		bool isMoving = std::abs(cVelocity) > 50.0;

		dynamicControl = isMoving || remainingTime > -0.2;
				
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
	
	if (dynamicControlMode == DynamicControlMode::Travelling || dynamicControlMode  == DynamicControlMode::Approaching)
	{		
		double dynamicAngleError = cTargetAngle - cSensorAngle;
		double errorDerivative = (dynamicAngleError - lDynamicPositionError)/(cTime - lTime);
		lDynamicPositionError = dynamicAngleError;
		
		double velocityCorrection = dynamicAngleError * config->velocityCorrectionProportionalGain + errorDerivative * config->velocityCorrectionDerivativeGain;
		
		cTargetVelocity = cPlanTargetVelocity+velocityCorrection;
		
		double finalAngleError = motionPlan->finalAngle - cSensorAngle;

		//Approaching, check for shutdown time
		if (TimeUtil::timeUntil(motionPlan->endTime) < 0.2 && std::abs(finalAngleError) < config->approachDistanceThreshold)
		{
			dynamicControlMode = DynamicControlMode::Approaching;
			cTargetAngle = motionPlan->finalAngle;
			doPositionControl();			
		}
		else
		{
			doSpeedControl();
		}
	}
	else if (dynamicControlMode == DynamicControlMode::Stopping)
	{
		if (useBrakeToStop)
			commandDriver(0,DriverMode::Brake);
		else
			commandDriver(0,DriverMode::Coast);
	}
}


void PredictiveJointController::doStaticControl()
{
	doPositionControl();
}

void PredictiveJointController::doPositionControl()
{
	const double kP = config->get("PositionControl.Kp");
	const double kD = config->get("PositionControl.Kd");
	const double maxVelocity = config->get("PositionControl.MaxVelocity");
	
	double error = cTargetAngle - cSensorAngle;
	
	if (abs(error) < config->maxSetpointError)
		error = 0;
	
	double d_error = 0; //(error - (lTargetAngle - lSensorAngle))/(cTime - lTime);
	
	cTargetVelocity = error*kP + d_error * kD;
	cTargetVelocity = min(cTargetVelocity,maxVelocity);
	cTargetVelocity = max(cTargetVelocity,-maxVelocity);
	
	dynamicControlMode = DynamicControlMode::Approaching;
	doSpeedControl();
}

void PredictiveJointController::doSpeedControl()
{
	if (speedControlState == SpeedControlState::Measuring)
	{
//		if (TimeUtil::timeSince(speedControlMeasureStart) > MinSpeedControlMeasureDelay && cVelocityApproximationError < 0.7) //MinVelocityRValue)
//		{
//			speedControlState = SpeedControlState::Adjusting;
//		}
//		else
		if (TimeUtil::timeSince(speedControlMeasureStart) > config->maxVelocityMeasureDelay)
		{
			speedControlState = SpeedControlState::Adjusting;
		}
		else
		{
			commandDriver(servoModel->getVoltageForTorqueSpeed(cControlTorque,cTargetVelocity),DriverMode::TMVoltage);
		}
	}
	
	cExpectedVelocity = servoModel->getSpeedForTorqueVoltage(cControlTorque,cVoltage);
	double cVelocityError = cExpectedVelocity - cVelocity;
	velocityErrorIntegral += cVelocityError * (cTime - lTime);
	double dVError = (cVelocityError - lVelocityError)/(cTime - lTime);

	if (!cDriverCommanded && speedControlState == SpeedControlState::Adjusting)
	{
		double torqueCorrection;

		torqueCorrection = -(cVelocityError * config->speedControlProportionalGain +
									dVError * config->speedControlDerivativeGain)/servoModel->getTorqueSpeedSlope();
		
		 
		cControlTorque += torqueCorrection;
		
		if (cVelocity == 0 && config->nonLinearTorqueResponseEnabled)
		{
			double torqueOffset = config->nonLinearTorqueOffset * sgn(cControlTorque);
			commandDriver(servoModel->getVoltageForTorqueSpeed(cControlTorque+torqueOffset,cTargetVelocity),DriverMode::TMVoltage);
		}
		else
		{
			commandDriver(servoModel->getVoltageForTorqueSpeed(cControlTorque,cTargetVelocity),DriverMode::TMVoltage);
		}
		
		speedControlState = SpeedControlState::Measuring;
		TimeUtil::setNow(speedControlMeasureStart);
	}

	velocityErrorIntegral *= 0.95;
	lVelocityError = cVelocityError;
	
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
			{
				if ((int)config->get("InvertVoltage") == 0)
					driverCommand = DRV8830::buildCommand(nAppliedVoltage);
				else
					driverCommand = DRV8830::buildCommand(-nAppliedVoltage);
				
			}
			else if (nDriverMode == DriverMode::Coast)
				driverCommand = DRV8830::buildCommand(0, DriveMode::OFF);
			else
			{
				emergencyHalt("Invalid driver mode.");
				return;
			}
			
			if (cDriverCommand != driverCommand)
			{
				if (Configuration::AsyncDriverCommunication)
				{
					AsyncI2CSender::forBus(bus[servoModel->driverBus])->postMessage(I2CMessage(servoModel->driverAddress, 0, driverCommand));
				}
				else
				{
					bus[servoModel->driverBus]->selectAddress(servoModel->driverAddress);
					DRV8830::writeCommand(bus[servoModel->driverBus],driverCommand);
				}
				cDriverCommand = driverCommand;
			}
		}
	}
	catch (std::runtime_error & e)
	{
		emergencyHalt("Cannot commit command");
		cout << "Error occured during commit: " << e.what();
	}
}

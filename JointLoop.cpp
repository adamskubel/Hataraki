#include "JointLoop.hpp"
#include <cmath>

using namespace std;

JointLoop::JointLoop(I2CBus * bus, cJSON * jointItem, double _samplePeriod)
{	
	this->samplePeriod = _samplePeriod;
	
	this->state = JointState::New;

	this->bus = bus;
	this->driverAddress = cJSON_GetObjectItem(jointItem,"DriverAddress")->valueint;
	this->sensorAddress= cJSON_GetObjectItem(jointItem,"SensorAddress")->valueint;
	this->jointName = cJSON_GetObjectItem(jointItem,"Name")->valuestring;
		
	this->setAngleRange(
			cJSON_GetObjectItem(jointItem,"MinAngle")->valuedouble,
			cJSON_GetObjectItem(jointItem,"MaxAngle")->valuedouble);

	this->motorConstant = cJSON_GetObjectItem(jointItem,"MotorConstant")->valuedouble;

	this->zeroPosition = AS5048::degreesToSteps(cJSON_GetObjectItem(jointItem,"ZeroPosition")->valuedouble);
	
	if (Configuration::CsvLoggingEnabled) {
		cout << "Opening csv log file for joint " << jointName << endl;
		csvLog.open("Data_" + jointName + ".csv");
		csvLog << "Time,CurrentAngle,TargetAngle,VoltageTarget,VoltageActual,TargetSpeed,ActualSpeed" <<endl;

		taskCsvLog.open("SpikeTask_" + jointName + ".csv");			
		taskCsvLog << "Time,CurrentAngle,VoltageActual" << endl;
	}

	logWhileWaiting = false;
	targetJointVelocity = 0;
	runningTaskIndex = 0;
	minSettleTime = 0.1;
	maxSettleError = 50;	
	lastVoltageError = 1000;
	

	hasSettled = false;
	isSettling = false;
	canTravel = isTravelling = false;

	cJSON * jointSpeedControlItem = cJSON_GetObjectItem(jointItem,"SpeedController");

	speedController = new SpeedController(cJSON_GetObjectItem(jointSpeedControlItem,"Kp")->valuedouble,
											cJSON_GetObjectItem(jointSpeedControlItem,"Ki")->valuedouble,
											cJSON_GetObjectItem(jointSpeedControlItem,"Kd")->valuedouble,
											motorConstant,
											cJSON_GetObjectItem(jointSpeedControlItem,"FilterRC")->valuedouble);

	speedController->enableClosedLoopControl(cJSON_GetObjectItem(jointSpeedControlItem,"Enabled")->valueint == 1);
	
	configurePositionController(cJSON_GetObjectItem(jointItem,"PositionController"));
}



void JointLoop::configurePositionController(cJSON * controllerConfig) {
	this->kP = cJSON_GetObjectItem(controllerConfig,"Kp")->valuedouble;
	this->kI = cJSON_GetObjectItem(controllerConfig,"Ki")->valuedouble;
	this->kD = cJSON_GetObjectItem(controllerConfig,"Kd")->valuedouble;
	this->errorSumMaxMagnitude = cJSON_GetObjectItem(controllerConfig,"MaxErrorIntegralMagnitude")->valuedouble;

}

void JointLoop::setControllerCoefficients(double Kp, double Ki, double Kd) {
	this->kP = Kp;
	this->kI = Ki;
	this->kD = Kd;
}

string JointLoop::getJointName(){
	return jointName;
}

void JointLoop::initJoint()
{	
	errorSum = 0;
	lastAngle = 0;

	clock_gettime(CLOCK_REALTIME, &startTime);

	cout << "------------------------------------------------" << endl;
	cout << "Initializing joint '" << jointName.c_str() << "'" << endl;	
	cout << "Gains set to " << kP << "," << kI << "," << kD << ", MaxIntegral=" << errorSumMaxMagnitude << endl;

	//AS5048

	bus->selectAddress(sensorAddress);

	try 
	{
		int angle = transformAngle(AS5048::getSensorAngleSteps(bus));
		unsigned char agc = AS5048::getAutoGainValue(bus);
		unsigned char diagnosticFlags = AS5048::getDiagnosticFlags(bus);
		
		bool encoderError = false;
		if (!checkAngleRange(angle))	{
			cout << "Encoder is reporting an angle out of range. Status:" << endl;
			printStatus();
			encoderError = true;
		}

		if (!AS5048::isValidStatus(diagnosticFlags)) {
			cout << "Encoder is reporting an invalid condition. Status:" << endl;
			printStatus();
			AS5048::printDiagnosticFlags(diagnosticFlags);
			encoderError = true;
		}

		if (encoderError) {		
			state = JointState::Error;
		}
		else {
			 cout << "Encoder initialization successful! Status:" << endl;
			 printStatus();
			 lastAngle = angle;
			 cout << endl;
		}
	}
	catch (std::runtime_error & e)
	{
		cout << "Exception while trying to communicate with sensor: " << e.what() << endl;
		cout << "Joint is in error state and will not function." << endl;
		state = JointState::Error;
	}
	

	//DRV8830
	try 
	{
		bus->selectAddress(driverAddress);	
		unsigned char driverFaults = DRV8830::readFaultRegister(bus);

		if (!DRV8830::hasFault(driverFaults))
		{
			cout << "Driver is ready!" << endl;
		}
		else
		{
			cout << "Driver is in FAULT state. Register value = 0x" << hex << (int)driverFaults << endl;
			DRV8830::printFaultRegister(driverFaults);
			state = JointState::Error;
		}
	}
	catch (runtime_error & e)
	{
		cout << "Exception while trying to communicate with driver: " << e.what() << endl;
		cout << "Joint is in error state and will not function." << endl;
		state = JointState::Error;
	}

	//Result

	if (state == JointState::New)
	{
		cout << "Joint is now waiting for command." << endl;
		state = JointState::Waiting;
	}
	else
	{
		cout << "Initialization failed, joint function is disabled." << endl;
	}
	cout << "------------------------------------------------" << endl << endl;
}

void JointLoop::printStatus()
{
	bus->selectAddress(sensorAddress);
	int angle = AS5048::getSensorAngleSteps(bus);
	unsigned char agc = AS5048::getAutoGainValue(bus);
	unsigned char diagnosticFlags = AS5048::getDiagnosticFlags(bus);


	 cout << "Name = " << jointName
		 << ", State = " << state
		 << ", Angle = " << AS5048::stepsToDegrees(transformAngle(angle)) << "(Raw=" << AS5048::stepsToDegrees(angle) << ") "
		 << ", Gain = " << dec << (int)agc
		 << ",Diagnostics = 0x" << hex << (int)diagnosticFlags
		 << endl;
	
}

void validateAngle(int angle){

	if (angle < -AS5048::PI_STEPS || angle > AS5048::PI_STEPS)
		throw std::runtime_error("Angle must be between -180 and 180 degrees");
}

int JointLoop::transformAngle(int angle) {
	return MathUtil::offsetAngleSteps(angle,zeroPosition);
}


void JointLoop::setAngleRange(double _minAngle, double _maxAngle)
{
	this->minAngle = AS5048::degreesToSteps(_minAngle);
	this->maxAngle = AS5048::degreesToSteps(_maxAngle);

	validateAngle(minAngle);
	validateAngle(maxAngle);
}

void JointLoop::setTargetAngle(double targetAngleDegrees)
{
	if (state == JointState::Waiting || state == JointState::Active)
	{
		if (state == JointState::Waiting)
		{
			clock_gettime(CLOCK_REALTIME, &startTime);
			state = JointState::Active;			

			cout << "Joint '" << jointName << "' is now ACTIVE. " << endl;
		}
		isSettling = hasSettled = false;
		canTravel = true;
		isTravelling = false;
		targetAngleSteps = AS5048::degreesToSteps(targetAngleDegrees);
	}
	else
		throw std::runtime_error("Joint is not in a valid state to accept commands.");
}

void JointLoop::setTargetJointVelocity(double _targetJointVelocity)
{
	this->targetJointVelocity = AS5048::radiansToSteps(_targetJointVelocity);
}

bool JointLoop::checkAngleRange(int angle)
{
	if (minAngle > maxAngle && (angle < minAngle && angle > maxAngle))
	{
		return false;
	}
	else if (minAngle < maxAngle && (angle < minAngle || angle > maxAngle))
	{
		return false;
	}
	else
		return true;
}

bool JointLoop::checkCommandValid(double angle)
{
	if (state == JointState::Waiting || state == JointState::Active)
	{
		int angleSteps = AS5048::degreesToSteps(angle);
		return checkAngleRange(angleSteps);
	}
	else
		return false;
}


void JointLoop::pauseJoint()
{
	switch (state) {
	case JointState::Waiting:	
	case JointState::New:
	case JointState::EmergencyHalt:
	case JointState::Error:
		break;
	case JointState::RunningTask:
	case JointState::Active:
		state = JointState::EmergencyHalt; //Make sure joint gets stopped if this fails
		bus->selectAddress(driverAddress);
		DRV8830::writeVoltageMode(bus,0,DriveMode::BRAKE);
		state = JointState::Waiting;
		break;
	}
}

void JointLoop::executeSpike(double voltage,int spikeLength, int zeroLength, int repeatCount)
{
	if (state == JointState::Waiting) {
		
		int cycleLength = spikeLength + zeroLength;
		int totalSteps = repeatCount * (cycleLength);
		runningTask = [this,voltage,spikeLength,cycleLength,totalSteps](int step, double totalTaskTime) -> bool {
			
			if (step >= totalSteps) {				
				DRV8830::writeVoltageMode(bus,0,DriveMode::BRAKE);
				return true;
			}

			double vDriver = 0;
			int cycleStep = step % cycleLength;
			//Spike on
			if (cycleStep == 0) {
				bus->selectAddress(driverAddress);
				DRV8830::writeVoltage(bus,voltage);
				vDriver = voltage;
			}
			else if (cycleStep < spikeLength) 
				vDriver = voltage;
			else
				vDriver = 0.1;

			if (cycleStep == spikeLength) {
				DRV8830::writeVoltage(bus,0.1); //nonzero value to keep PWM active
			}
						
			bus->selectAddress(sensorAddress);
			int rawAngle = AS5048::getSensorAngleSteps(bus);
			int angle = transformAngle(rawAngle);
			
			taskCsvLog << totalTaskTime << "," << angle << "," << DRV8830::getNearestVoltage(vDriver)  << endl;
			return false;
		};

		logWhileWaiting = true;
		runningTaskIndex = 0;
		state = JointState::RunningTask;
	}
}

double JointLoop::getCurrentAngle()
{
	return AS5048::stepsToDegrees(lastAngle);
}

double JointLoop::getMaxAngle()
{
	return AS5048::stepsToDegrees(maxAngle);
}

double JointLoop::getMinAngle() 
{
	return AS5048::stepsToDegrees(minAngle);
}

double JointLoop::getMaxJointVelocity()
{
	double maxVoltage = DRV8830::stepsToVoltage(DRV8830::MaxVoltageStep);
	return AS5048::stepsToRadians(maxVoltage*motorConstant);
}


bool JointLoop::isSettled()
{
	return hasSettled;
}


void JointLoop::checkSettling(int error, double voltageError)
{
	double totalTime = timeSince(startTime);
	if (DRV8830::getNearestVoltage(voltageError) == 0 || (MathUtil::abs(error) < maxSettleError)){

		if (isSettling) {
			double settleTime = totalTime - settlingStartTime;
			if (settleTime > minSettleTime)
			{
				isSettling = false;
				hasSettled = true;
			}
		}
		else
		{
			isSettling = true;
			settlingStartTime = totalTime;
		}
	}
}

double JointLoop::getMinTravelVoltageError(){
	
	return DRV8830::stepsToVoltage(DRV8830::MaxVoltageStep)*1.5;
}

double JointLoop::timeSince(struct timespec & sinceTime){
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	return MathUtil::getTimeDelta(sinceTime,now);
}

void JointLoop::run()
{
	if (state == JointState::New) 
	{
		initJoint();
	}
	if (state == JointState::Waiting) {

		if (logWhileWaiting) {
				
			double time = timeSince(startTime);
			
			bus->selectAddress(sensorAddress);
			int rawAngle = AS5048::getSensorAngleSteps(bus);
			int angle = transformAngle(rawAngle);
			
			taskCsvLog << time << "," << angle << "," << "0" << endl;
		}
	}
	else if (state == JointState::EmergencyHalt)
	{
		try {
			bus->selectAddress(driverAddress);
			DRV8830::writeVoltageMode(bus,0,DriveMode::OFF);
			state = JointState::Error;
			cout << "Joint " << jointName << " successfully halted and placed in Error state." << endl;
		}
		catch (std::runtime_error & e)
		{
			cout << "FATAL ERROR: CANNOT STOP JOINT. MANUAL INTERVENTION REQUIRED IMMEDIATELY!" << endl;
		}
		
	}
	else if (state == JointState::RunningTask) {
		double totalTaskTime = timeSince(startTime);
		if (runningTask(runningTaskIndex++, totalTaskTime))
		{			
			pauseJoint();
		}
	}
	else if (state == JointState::Active)
	{		
		try {
			
			bus->selectAddress(sensorAddress);
			int rawAngle = AS5048::getSensorAngleSteps(bus);
			int angle = transformAngle(rawAngle);
		
			double elapsedSeconds = timeSince(lastRunTime);
			clock_gettime(CLOCK_REALTIME, &lastRunTime);

			double totalTime = timeSince(startTime);

			speedController->updateAngle(elapsedSeconds,angle);
		
			if (!checkAngleRange(angle))	
			{
				state = JointState::EmergencyHalt;
				bus->selectAddress(driverAddress);
				DRV8830::writeVoltageMode(bus,0,DriveMode::OFF);
				state = JointState::Error;
				cout << "Joint '" << jointName.c_str() << "' has entered fault state due to angle out of range. Angle = " << AS5048::stepsToDegrees(angle) << " (Raw=" << AS5048::stepsToDegrees(rawAngle) << ")" << endl;
			}
			else
			{
				double angleDelta = AS5048::getAngleError(angle,lastAngle);
				
				if (MathUtil::abs(angleDelta) > AS5048::PI_STEPS*0.9) {
					
					state = JointState::EmergencyHalt;
					bus->selectAddress(driverAddress);
					DRV8830::writeVoltageMode(bus,0,DriveMode::OFF);
					state = JointState::Error;
					cout << "Joint '" << jointName << " experienced an angle flip. Halting motion. Angle = "
						<< AS5048::stepsToDegrees(angle) << " (Raw=" << AS5048::stepsToDegrees(rawAngle) << ")"
						<< " LastAngle = " << AS5048::stepsToDegrees(lastAngle) << endl;
					
				}
				
				int error = AS5048::getAngleError(angle,targetAngleSteps);

				errorSum += ((double)error)*elapsedSeconds;

				int lastError = AS5048::getAngleError(lastAngle,targetAngleSteps);
				double errorDerivative = ((double)(error-lastError))/elapsedSeconds;
								
				errorSum = std::min(errorSum,errorSumMaxMagnitude);
				errorSum = std::max(-errorSumMaxMagnitude,errorSum);
			
				double voltageError = (kP*error + (kI*errorSum) + (kD*errorDerivative))/(motorConstant*samplePeriod);
				
				if (MathUtil::abs(voltageError) > getMinTravelVoltageError())
				{
					if (canTravel)
					{
						speedController->setTargetSpeed(targetJointVelocity*MathUtils::sgn(voltageError));
						canTravel = false;
						isTravelling = true;
					}
				}
				else
				{
					isTravelling = false;
				}

				if (isTravelling && targetJointVelocity != 0)
					voltageError = speedController->getNextVoltage();

				csvLog << totalTime << "," << angle << "," << targetAngleSteps << "," 
					<< voltageError << "," << DRV8830::getNearestVoltage(voltageError)  << ","
					<< speedController->getTargetSpeed() << "," << speedController->getLastSpeed() 
					<< endl;

				checkSettling(error,voltageError);

				if (MathUtil::abs(voltageError - lastVoltageError) > 0.001) {
					bus->selectAddress(driverAddress);
					DRV8830::writeVoltage(bus,voltageError);	
					lastVoltageError = voltageError;
				}
			}
			
			lastAngle = angle;
		}
		catch (std::runtime_error & e)
		{
			cout << "Exception occurred during control of joint " << jointName << ". Error = " << e.what() << endl;
			cout << "Joint will be placed in emergency state to prevent damage." << endl;
			state = JointState::EmergencyHalt;
		}
	}
}

void JointLoop::requestEmergencyHalt()
{
	state = JointState::EmergencyHalt;
}

void JointLoop::shutdown()
{		
	if (state == JointState::Active || state == JointState::EmergencyHalt) 
	{
		try
		{
			bus->selectAddress(driverAddress);
			DRV8830::writeVoltageMode(bus,0,DriveMode::OFF);
		}
		catch (std::runtime_error & e)
		{
			cout << "FATAL ERROR: CANNOT STOP JOINT '" << jointName << "'. MANUAL INTERVENTION REQUIRED IMMEDIATELY!" << e.what() << endl;;
		}
	}

	csvLog.close();
}

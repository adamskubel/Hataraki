#include "SpeedController.hpp"

SpeedController::SpeedController(double _Kp, double _Ki, double _Kd, double _motorConstant, double _filterRC) {
	this->Kp = _Kp;
	this->Ki = _Ki;
	this->Kd = _Kd;
	this->motorConstant = _motorConstant;
	this->lowpassFilterRC = _filterRC;

	closedLoopControlEnabled = true;
	setTargetSpeed(0);

	if (_filterRC > 1.0)
		movingAverage = new SimpleMovingAverage((unsigned int)_filterRC);
	else
		movingAverage = NULL;
}


void SpeedController::updateAngle(double elapsedTime, int newAngle){

	if (closedLoopControlActive) {

		double speed = (newAngle-lastAngle)/elapsedTime;
				
		if (movingAverage)
		{
			movingAverage->add(speed);
			speed = movingAverage->avg();
		}
		else
			speed = MathUtil::lowpass(lastSpeed,speed,lowpassFilterRC,elapsedTime);

		double error = targetSpeed - speed;

		errorSum += error * elapsedTime;

		double lastError = targetSpeed - lastSpeed;
		double errorDiff = (error-lastError) / elapsedTime;
		lastSpeed = speed;
		
		double voltageAdjust = (Kp*error + Ki*errorSum + Kd*errorDiff)/motorConstant;
	
		currentVoltage += voltageAdjust;
	}
	else 
	{
		currentVoltage = targetSpeed/motorConstant;
		
		if (MathUtil::abs(currentVoltage) > 0 && MathUtil::abs(currentVoltage) < DRV8830::MinDriverVoltage)
			currentVoltage = DRV8830::MinDriverVoltage*MathUtils::sgn(currentVoltage);

		closedLoopControlActive = closedLoopControlEnabled;
	}

	lastAngle = newAngle;
}

double SpeedController::getNextVoltage() {

	return currentVoltage;
}


void SpeedController::setTargetSpeed(double _targetSpeed) {

	this->targetSpeed = _targetSpeed;
	errorSum = 0;
	closedLoopControlActive = false;
	lastAngle = 0;

}

double SpeedController::getTargetSpeed() {
	return targetSpeed;
}

double SpeedController::getLastSpeed() {
	return lastSpeed;
}

void SpeedController::enableClosedLoopControl(bool enabled) {

	this->closedLoopControlEnabled = enabled;
}



				
				//if (voltageError > maxVoltage)
				//	voltageError = maxVoltage;
				//else if (voltageError < -maxVoltage)
				//	voltageError = -maxVoltage;
				
				//if (MathUtil::abs(voltageError) < DRV8830::MinDriverVoltage) {
				//	//Speed control
				//	
				//	double vHigh = 0.49, vLow = 0;
				//	double alpha = (MathUtil::abs(voltageError)-vHigh)/(vLow-vHigh);

				//	int cycleLength = 12;
				//	int lowCycles = (int)std::round(alpha * (double)cycleLength);					
				//	int highCycles = cycleLength - lowCycles;

				//	//if (highCycles > 0)
				//		//highCycles += 2;
				//	
				//	if ((speedControlIndex++) % cycleLength < highCycles)
				//		voltageError = vHigh*MathUtils::sgn(voltageError);
				//	else
				//		voltageError = 0.3*MathUtils::sgn(voltageError);
				//	
				//} 
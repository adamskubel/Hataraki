#include "ServoModel.hpp"


double ServoModel::getNoLoadSpeedForVoltage(double voltage) 
{
	double sign = sgn(voltage);
	double speed = (voltage - sign*motor.noLoadCurrent*motor.armatureResistance)/motor.backEmfConstant;

	if (sign > 0)
		return std::max<double>(speed,0);
	else
		return std::min<double>(0,speed);
}

double ServoModel::getTorqueSpeedSlope() 
{
	// return -motor.noLoadSpeed / (motor.stallTorque - motor.noLoadTorque);
	return -motor.armatureResistance/(motor.torqueConstant*motor.backEmfConstant);
}	

double ServoModel::getTorqueForVoltageSpeed(double voltage, double speed)
{
	//return ((speed - getNoLoadSpeedForVoltage(voltage))/torqueSpeedSlope) + (sign*motor.noLoadTorque);
	return ((voltage - (motor.backEmfConstant*speed))/motor.armatureResistance - (sgn(voltage)*motor.noLoadCurrent))*motor.torqueConstant;
}

double ServoModel::getVoltageForTorqueSpeed(double torque, double speed)
{
	//return (speed - ((torque+(sign*motor.noLoadTorque))*torqueSpeedSlope))*motor.backEmfConstant;
	//return motor.armatureResistance*((torque+motor.noLoadTorque)/motor.torqueConstant) + speed * motor.backEmfConstant;
	return motor.armatureResistance*(torque/motor.torqueConstant + sgn(speed)*motor.noLoadCurrent) + speed * motor.backEmfConstant;
}

double ServoModel::getSpeedForTorqueVoltage(double torque, double voltage)
{		
	//return getNoLoadSpeedForVoltage(voltage) + torqueSpeedSlope*(torque-(sign*motor.noLoadTorque));
	return (voltage - motor.armatureResistance*(torque/motor.torqueConstant + sgn(voltage)*motor.noLoadCurrent))/motor.backEmfConstant;		
}
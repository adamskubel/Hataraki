#include "ServoModel.hpp"


double ServoModel::getTorqueSpeedSlope() 
{
	return -motor.armatureResistance/(motor.torqueConstant*motor.backEmfConstant);
}	

double ServoModel::getTorqueForVoltageSpeed(double voltage, double speed)
{
	return ((voltage - (motor.backEmfConstant*speed))/motor.armatureResistance - (sgn(voltage)*motor.noLoadCurrent))*motor.torqueConstant;
}

double ServoModel::getVoltageForTorqueSpeed(double torque, double speed)
{
	//torque *= -1.0;
	return motor.armatureResistance*(torque/motor.torqueConstant + sgn(speed)*motor.noLoadCurrent) + speed * motor.backEmfConstant;
}

double ServoModel::getSpeedForTorqueVoltage(double torque, double voltage)
{
	//torque *= -1.0;
	return (voltage - motor.armatureResistance*(torque/motor.torqueConstant + sgn(voltage)*motor.noLoadCurrent))/motor.backEmfConstant;		
}
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



ControllerConfig::ControllerConfig(cJSON * _rawConfig)
{
	this->rawConfig = _rawConfig;
	
	speedControlProportionalGain = Configuration::getInstance().getObject(rawConfig,"SpeedControl.ProportionalGain")->valuedouble;
	speedControlIntegralGain = Configuration::getInstance().getObject(rawConfig,"SpeedControl.IntegralGain")->valuedouble;
	speedControlDerivativeGain = Configuration::getInstance().getObject(rawConfig,"SpeedControl.DerivativeGain")->valuedouble;
	
	maxAcceleration = AS5048::degreesToSteps(Configuration::getInstance().getObject(rawConfig,"MaxAcceleration")->valuedouble);
	
	velocityCorrectionProportionalGain = Configuration::getInstance().getObject(rawConfig,"DynamicController.VelocityKPForPositionCorrection")->valuedouble;
	velocityCorrectionDerivativeGain = Configuration::getInstance().getObject(rawConfig,"DynamicController.VelocityKDForPositionCorrection")->valuedouble;
	
	approachDistanceThreshold = AS5048::degreesToSteps(Configuration::getInstance().getObject(rawConfig,"DynamicController.SetpointApproachDistanceThreshold")->valuedouble);
	
	positionHistorySize = Configuration::getInstance().getObject(rawConfig,"SpeedControl.HistoryLength")->valueint;
	maxSetpointError = get("SetpointPrecisionSteps");
	
	useTargetFeedback = false;// (bool)get("SpeedControl.UseTargetFeedback");
	
	maxVelocityMeasureDelay = get("SpeedControl.MaxMeasureDelay");
	
	samplesPerUpdate = (int)get("SamplesPerUpdate");
	
	nonLinearTorqueOffset = get("NonLinearTorque.StartingTorque");
	nonLinearTorqueResponseEnabled = (bool)get("NonLinearTorque.Enabled");
}
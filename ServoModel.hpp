#ifndef HATARAKI_BASICMOTION_MODEL_SERVOMODEL_HPP_
#define HATARAKI_BASICMOTION_MODEL_SERVOMODEL_HPP_

#include "cJSON.h"
#include "Configuration.hpp"


struct GearboxModel {

	double 
		ratio,  //Scale factor from output to input (1000,298,etc)
		lostMotion, //Lost motion in radians
		efficiency, //Ratio of output torque to input torque
		viscousFriction; //Rotational friction as a function of speed: N*m/(rad/s)

	GearboxModel(cJSON * rawConfig) {

		Configuration::AssertConfigExists(rawConfig,"GearboxModel");

		ratio = cJSON_GetObjectItem(rawConfig,"Ratio")->valuedouble;
		lostMotion = MathUtil::degreesToRadians(cJSON_GetObjectItem(rawConfig,"LostMotion")->valuedouble);
		efficiency = cJSON_GetObjectItem(rawConfig,"Efficiency")->valuedouble;
		viscousFriction = cJSON_GetObjectItem(rawConfig,"ViscousFriction")->valuedouble;
	}
	
};

class MotorModel {

public:
	double 
		torqueConstant, //	(N*m)/amp
		backEmfConstant, //	V/RPM
		armatureResistance, //Ohms
		stallCurrent, //Amps
		stallTorque, // N*m
		noLoadTorque, //Friction torque
		noLoadCurrent,
		noLoadSpeed; //RPM

	MotorModel(cJSON * rawConfig) {
		
		Configuration::AssertConfigExists(rawConfig,"MotorModel");

		torqueConstant = cJSON_GetObjectItem(rawConfig,"TorqueConstant")->valuedouble;
		backEmfConstant = cJSON_GetObjectItem(rawConfig,"BackEmfConstant")->valuedouble;
		armatureResistance = cJSON_GetObjectItem(rawConfig,"ArmatureResistance")->valuedouble;
		//stallCurrent = cJSON_GetObjectItem(rawConfig,"StallCurrent")->valuedouble;
		stallTorque = cJSON_GetObjectItem(rawConfig,"StallTorque")->valuedouble;
		//noLoadCurrent = cJSON_GetObjectItem(rawConfig,"NoLoadCurrent")->valuedouble;
		noLoadTorque = cJSON_GetObjectItem(rawConfig,"NoLoadTorque")->valuedouble;
		noLoadSpeed = cJSON_GetObjectItem(rawConfig,"NoLoadSpeed")->valuedouble;
		
		noLoadCurrent = noLoadTorque/torqueConstant;
		stallCurrent = stallTorque/torqueConstant;
	}
	


};

class ServoModel {
		
public:
	GearboxModel gearbox;
	MotorModel motor;

	double driverDelay, sensorDelay;
	
	int driverAddress;
	int sensorAddress;

	
	ServoModel(cJSON * rawConfig) :
		gearbox(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"Gearbox")->valuestring)),
		motor(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"Motor")->valuestring))
	{
		sensorAddress = cJSON_GetObjectItem(rawConfig, "SensorAddress")->valueint;
		driverAddress = cJSON_GetObjectItem(rawConfig, "DriverAddress")->valueint;
	}

	//These functions are here because they incorporate both gearbox and motor parameters
	double getNoLoadSpeedForVoltage(double voltage) 
	{
		return (voltage - motor.noLoadCurrent*motor.armatureResistance)/motor.backEmfConstant;
	}
		
	double getTorqueSpeedSlope() 
	{
		// return -motor.noLoadSpeed / (motor.stallTorque - motor.noLoadTorque);
		return -motor.armatureResistance/(motor.torqueConstant*motor.backEmfConstant);
	}	

	double getTorqueForVoltageSpeed(double voltage, double speed)
	{
		double torqueSpeedSlope = getTorqueSpeedSlope();
		return ((speed - getNoLoadSpeedForVoltage(voltage))/torqueSpeedSlope) + motor.noLoadTorque;
	}

	double getVoltageForTorqueSpeed(double torque, double speed)
	{
		double torqueSpeedSlope = getTorqueSpeedSlope();
		return (speed - ((torque-motor.noLoadTorque)*torqueSpeedSlope))*motor.backEmfConstant;
	}

	double getSpeedForTorqueVoltage(double torque, double voltage)
	{		
		double torqueSpeedSlope = getTorqueSpeedSlope();
		return getNoLoadSpeedForVoltage(voltage) + torqueSpeedSlope*torque;
	}

};

class JointModel {
	
public:
	ServoModel servoModel;

	double maxAngle, minAngle;
	double sensorZeroPosition;

	std::string name;

	JointModel(cJSON * rawConfig) :
		servoModel(cJSON_GetObjectItem(rawConfig,"Servo"))
	{
		
	}

};

struct SegmentModel {


	SegmentModel(cJSON * rawConfig) {
		
	}

};

struct ArmModel {
	


};



#endif
#ifndef HATARAKI_BASICMOTION_MODEL_SERVOMODEL_HPP_
#define HATARAKI_BASICMOTION_MODEL_SERVOMODEL_HPP_

//#include "cJSON.h"
#include "Configuration.hpp"
#include "MathUtils.hpp"
#include "AS5048.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"


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
		backEmfConstant, //	V/RPM -> V/(steps/second)
		armatureResistance, //Ohms
		stallCurrent, //Amps
		stallTorque, // N*m
		noLoadTorque, //Friction torque
		noLoadCurrent,
		noLoadSpeed; //RPM -> steps/second

	MotorModel(cJSON * rawConfig) {
		
		Configuration::AssertConfigExists(rawConfig,"MotorModel");

		torqueConstant = cJSON_GetObjectItem(rawConfig,"TorqueConstant")->valuedouble;
		backEmfConstant = MathUtil::stepsPerSecondToRPM(cJSON_GetObjectItem(rawConfig,"BackEmfConstant")->valuedouble);
		armatureResistance = cJSON_GetObjectItem(rawConfig,"ArmatureResistance")->valuedouble;
		stallTorque = cJSON_GetObjectItem(rawConfig,"StallTorque")->valuedouble;
		noLoadTorque = cJSON_GetObjectItem(rawConfig,"NoLoadTorque")->valuedouble;
		noLoadSpeed = MathUtil::rpmToStepsPerSecond(cJSON_GetObjectItem(rawConfig,"NoLoadSpeed")->valuedouble);
		
		noLoadCurrent = noLoadTorque/torqueConstant;
		stallCurrent = stallTorque/torqueConstant;
	}
	


};

class ServoModel {
		
public:
	GearboxModel gearbox;
	MotorModel motor;

	double driverDelay, sensorDelay;
	double maxDriverVoltage;
	
	int driverAddress;
	int sensorAddress;

	
	ServoModel(cJSON * rawConfig) :
		gearbox(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"Gearbox")->valuestring)),
		motor(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"Motor")->valuestring))
	{		
		Configuration::AssertConfigExists(rawConfig,"ServoModel");

		sensorAddress = cJSON_GetObjectItem(rawConfig, "SensorAddress")->valueint;
		driverAddress = cJSON_GetObjectItem(rawConfig, "DriverAddress")->valueint;
		driverDelay = cJSON_GetObjectItem(rawConfig,"DriverDelay")->valuedouble;
		sensorDelay = cJSON_GetObjectItem(rawConfig,"SensorDelay")->valuedouble;
		maxDriverVoltage = cJSON_GetObjectItem(rawConfig,"MaxDriverVoltage")->valuedouble;
	}

	//These functions are here because they incorporate both gearbox and motor parameters
	double getNoLoadSpeedForVoltage(double voltage) 
	{
		double sign = MathUtils::sgn<double>(voltage);
		double speed = (voltage - sign*motor.noLoadCurrent*motor.armatureResistance)/motor.backEmfConstant;
		
		if (sign > 0)
			return std::max<double>(speed,0);
		else
			return std::min<double>(0,speed);
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
		double sign = MathUtils::sgn<double>(speed);

		double torqueSpeedSlope = getTorqueSpeedSlope();
		return (speed - ((torque-(sign*motor.noLoadTorque))*torqueSpeedSlope))*motor.backEmfConstant;
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
	
	vmath::Vector3d axisOfRotation;

	int index;

	JointModel(cJSON * rawConfig) :
		servoModel(cJSON_GetObjectItem(rawConfig,"Servo"))
	{		
		Configuration::AssertConfigExists(rawConfig,"JointModel");
				
		name = std::string(cJSON_GetObjectItem(rawConfig,"Name")->valuestring);

		maxAngle = AS5048::degreesToSteps(cJSON_GetObjectItem(rawConfig,"MaxAngle")->valuedouble);
		minAngle = AS5048::degreesToSteps(cJSON_GetObjectItem(rawConfig,"MinAngle")->valuedouble);
		sensorZeroPosition = AS5048::degreesToSteps(cJSON_GetObjectItem(rawConfig,"ZeroPosition")->valuedouble);

		axisOfRotation = Configuration::getVectorFromJSON(cJSON_GetObjectItem(rawConfig,"AxisOfRotation"));
	}

};

//Rigid bodies connecting joints
struct SegmentModel {
	
	double mass; //Kg
	double boneLength; 

	vmath::Vector3d centerOfMass; //Relative to joint origin	
	
	SegmentModel(cJSON * rawConfig) {
		
		mass = cJSON_GetObjectItem(rawConfig,"Mass")->valuedouble;
		centerOfMass = Configuration::getVectorFromJSON(cJSON_GetObjectItem(rawConfig,"CenterOfMass"));
		boneLength = cJSON_GetObjectItem(rawConfig,"BoneLength")->valuedouble;
	}

};

struct ArmModel {
	
	std::vector<JointModel> joints;
	std::vector<SegmentModel> segments;

	ArmModel(cJSON * rawConfig) 
	{
		cJSON * jointArray = cJSON_GetObjectItem(rawConfig,"Joints");
		cJSON * segmentArray = cJSON_GetObjectItem(rawConfig,"Segments");

		for (int i=0;i<cJSON_GetArraySize(jointArray);i++)
		{
			joints.push_back(JointModel(Configuration::getInstance().getObject(cJSON_GetArrayItem(jointArray,i)->valuestring)));
			joints.back().index = i;
		}
		
		for (int i=0;i<cJSON_GetArraySize(segmentArray);i++)
			segments.push_back(SegmentModel(Configuration::getInstance().getObject(cJSON_GetArrayItem(segmentArray,i)->valuestring)));
		
	}

};



#endif
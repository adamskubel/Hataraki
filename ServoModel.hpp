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
		lostMotion = AS5048::degreesToSteps(cJSON_GetObjectItem(rawConfig,"LostMotion")->valuedouble);
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
		noLoadCurrent = cJSON_GetObjectItem(rawConfig,"NoLoadCurrent")->valuedouble;
		
		noLoadTorque = torqueConstant * noLoadCurrent;

		stallCurrent = stallTorque/torqueConstant;
	}
	


};

class ControllerConfig {

public:
	double speedControlProportionalGain;
	double speedControlIntegralGain;
	double speedControlDerivativeGain;

	std::vector<double> gravityFlipVoltagePattern, externalDisturbanceFlipVoltagePattern, startVoltagePattern;

	bool externalDisturbanceFlipTriggerEnabled;
	bool gravityFlipTriggerEnabled;
	bool motionStartEnabled;
	bool useTargetFeedback;
	bool stepControlEnabled;

	const bool savitzyGolayFilteringEnabled = false;
	const int savitzyGolayWindowSize = 5;
	const int savitzyGolayPolyDegree = 5;
	
	double velocityCorrectionProportionalGain,velocityCorrectionDerivativeGain;
	double approachVelocity;
	double approachDistanceThreshold;
	double maxSetpointError;
	double maxAcceleration;

	int positionHistorySize;

	cJSON * rawConfig;
	
	double get(std::string key)
	{
		return Configuration::getInstance().getObject(rawConfig,key)->valuedouble;
	}


	ControllerConfig(cJSON * _rawConfig) 
	{
		this->rawConfig = _rawConfig;

		speedControlProportionalGain = Configuration::getInstance().getObject(rawConfig,"SpeedControl.ProportionalGain")->valuedouble;
		speedControlIntegralGain = Configuration::getInstance().getObject(rawConfig,"SpeedControl.IntegralGain")->valuedouble;
		speedControlDerivativeGain = Configuration::getInstance().getObject(rawConfig,"SpeedControl.DerivativeGain")->valuedouble;

		gravityFlipVoltagePattern = Configuration::getVoltagePatternFromJSON(cJSON_GetObjectItem(rawConfig,"GravityFlipPattern"));
		
		startVoltagePattern = Configuration::getVoltagePatternFromJSON(Configuration::getInstance().getObject(rawConfig,"MotionStart.VoltagePattern"));
		motionStartEnabled = get("MotionStart.Enabled") != 0.0;

		externalDisturbanceFlipVoltagePattern = Configuration::getVoltagePatternFromJSON(cJSON_GetObjectItem(rawConfig,"ExternalDisturbanceFlipPattern"));

		externalDisturbanceFlipTriggerEnabled = cJSON_GetObjectItem(rawConfig,"ExternalDisturbanceFlipTriggerEnabled")->valueint != 0;
		gravityFlipTriggerEnabled = cJSON_GetObjectItem(rawConfig,"GravityFlipTriggerEnabled")->valueint != 0;

		maxAcceleration = AS5048::degreesToSteps(Configuration::getInstance().getObject(rawConfig,"MaxAcceleration")->valuedouble);

		velocityCorrectionProportionalGain = Configuration::getInstance().getObject(rawConfig,"DynamicController.VelocityKPForPositionCorrection")->valuedouble;
		velocityCorrectionDerivativeGain = Configuration::getInstance().getObject(rawConfig,"DynamicController.VelocityKDForPositionCorrection")->valuedouble;
		approachVelocity = AS5048::degreesToSteps(Configuration::getInstance().getObject(rawConfig,"DynamicController.SetpointApproachVelocity")->valuedouble);
		approachDistanceThreshold = AS5048::degreesToSteps(Configuration::getInstance().getObject(rawConfig,"DynamicController.SetpointApproachDistanceThreshold")->valuedouble);
		
		positionHistorySize = Configuration::getInstance().getObject(rawConfig,"SpeedControl.HistoryLength")->valueint;
		maxSetpointError = get("SetpointPrecisionSteps");
		
		useTargetFeedback = (bool)get("SpeedControl.UseTargetFeedback");
		
		stepControlEnabled = (bool)get("StepControl.Enabled");
	}


};

class ServoModel {
		
public:
	GearboxModel gearbox;
	MotorModel motor;
	ControllerConfig controllerConfig;

	double driverDelay, sensorDelay;
	double maxDriverVoltage;
	
	std::string driverBus, sensorBus;

	int driverAddress;
	int sensorAddress;

	double frictionTorque;
		
	ServoModel(cJSON * rawConfig) :
		gearbox(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"Gearbox")->valuestring)),
		motor(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"Motor")->valuestring)),
		controllerConfig(Configuration::getInstance().getObject(cJSON_GetObjectItem(rawConfig,"ControllerConfig")->valuestring))
	{		
		Configuration::AssertConfigExists(rawConfig,"ServoModel");

		sensorAddress = cJSON_GetObjectItem(rawConfig, "SensorAddress")->valueint;
		driverAddress = cJSON_GetObjectItem(rawConfig, "DriverAddress")->valueint;

		sensorBus = std::string(cJSON_GetObjectItem(rawConfig, "SensorBus")->valuestring);
		driverBus = std::string(cJSON_GetObjectItem(rawConfig, "DriverBus")->valuestring);

		driverDelay = cJSON_GetObjectItem(rawConfig,"DriverDelay")->valuedouble;
		sensorDelay = cJSON_GetObjectItem(rawConfig,"SensorDelay")->valuedouble;
		maxDriverVoltage = cJSON_GetObjectItem(rawConfig,"MaxDriverVoltage")->valuedouble;
		frictionTorque = cJSON_GetObjectItem(rawConfig,"FrictionTorque")->valuedouble;
	}

	//These functions are here because they incorporate both gearbox and motor parameters
	double getNoLoadSpeedForVoltage(double voltage);
		
	double getTorqueSpeedSlope();

	double getTorqueForVoltageSpeed(double voltage, double speed);

	double getVoltageForTorqueSpeed(double torque, double speed);

	double getSpeedForTorqueVoltage(double torque, double voltage);

};

class JointModel {

private:
	cJSON * rawConfig;
public:
	ServoModel servoModel;

	double maxAngle, minAngle;
	double sensorZeroPosition;
	bool continuousRotation;

	std::string name;
	
	vmath::Vector3d axisOfRotation;

	int index;
	

	double get(std::string key)
	{
		return Configuration::getInstance().getObject(rawConfig,key)->valuedouble;
	}

	JointModel(cJSON * rawConfig) :
		servoModel(cJSON_GetObjectItem(rawConfig,"Servo"))
	{		
		Configuration::AssertConfigExists(rawConfig,"JointModel");
		this->rawConfig = rawConfig;
				
		name = std::string(cJSON_GetObjectItem(rawConfig,"Name")->valuestring);
		
		continuousRotation = ((int)get("ContinuousRotation")) == 1;
		
		if (!continuousRotation)
		{
			maxAngle = AS5048::degreesToSteps(cJSON_GetObjectItem(rawConfig,"MaxAngle")->valuedouble);
			minAngle = AS5048::degreesToSteps(cJSON_GetObjectItem(rawConfig,"MinAngle")->valuedouble);
		}
		else
		{
			maxAngle = minAngle = 0;
		}

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
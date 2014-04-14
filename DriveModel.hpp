#ifndef HATARAKI_BASICMOTION_WHEEL_DRIVE_MODEL_HPP_
#define HATARAKI_BASICMOTION_WHEEL_DRIVE_MODEL_HPP_

#include "Configuration.hpp"
#include "ServoModel.hpp"


class WheelModel : public ConfigurationObject {

public:
	bool reverse;
	double distanceFromCenter;

	WheelModel(cJSON * rawConfig) : ConfigurationObject(rawConfig)
	{
		reverse = this->getBool("Reverse");
		distanceFromCenter = this->getDouble("DistanceFromCenter");
	}

};

class DriveModel : public ConfigurationObject {
	
public:
	double metersPerStep;	
	std::vector<JointModel> wheelJoints;
	std::vector<WheelModel> wheels;


	DriveModel(cJSON * rawConfig) : ConfigurationObject(rawConfig)
	{
		double diam = getDouble("WheelDiameter");
	//	cout << "Diam = " << diam << endl;
		metersPerStep = diam * (MathUtil::PI/16384.0);

		auto jointArray = getObject("Wheels");
		
		for (int i=0;i<cJSON_GetArraySize(jointArray);i++)
		{
			auto item = cJSON_GetArrayItem(jointArray,i);
			wheelJoints.push_back(JointModel(Configuration::getInstance().getObject(cJSON_GetObjectItem(item,"Joint")->valuestring)));
			wheelJoints.back().index = i;
			wheels.push_back(WheelModel(item));
		}

	}

	double stepsToMeters(double steps);
	double metersToSteps(double meters);

};

#endif
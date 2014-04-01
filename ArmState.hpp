#ifndef HATARAKI_BASICMOTION_ARM_STATE_HPP_
#define HATARAKI_BASICMOTION_ARM_STATE_HPP_

#include <vector>

#include "AS5048.hpp"

struct ArmState {

	std::vector<double> JointAngles;

public:	
	std::vector<double> getJointAnglesRadians()
	{
		std::vector<double> jointAnglesRadians(JointAngles.size());		
		std::transform(JointAngles.begin(),JointAngles.end(),jointAnglesRadians.begin(),AS5048::stepsToRadians);
		return jointAnglesRadians;
	}
};

#endif
#ifndef HATARAKI_BASICMOTION_ARM_STATE_HPP_
#define HATARAKI_BASICMOTION_ARM_STATE_HPP_

#include <vector>
#include <algorithm>

#include "IKFast.hpp"

#include "MathUtils.hpp"
#include "AS5048.hpp"
#include "OpSpaceState.hpp"

class ArmState {

private:
	std::vector<double> JointAngles;
	bool opSpaceStateComputed;
	OpSpaceState computedOperationalSpaceState;

	void calculateOpSpaceState();

public:
	ArmState();	
	ArmState (std::vector<double> initialAngles);	

	void setJointAngles(std::vector<double> initialAngles);

	std::vector<double> getJointAnglesRadians();
	std::vector<double> getJointAngles();
	
	OpSpaceState getOpSpaceState();


};

#endif
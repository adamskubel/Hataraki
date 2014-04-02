#include "ArmState.hpp"
#include "AsyncLogger.hpp"
using namespace std;
using namespace vmath;
using namespace ikfast;

ArmState::ArmState()
{	
	opSpaceStateComputed = false;
}

ArmState::ArmState (std::vector<double> initialAngles)
{
	JointAngles = initialAngles;
	opSpaceStateComputed = false;
}

vector<double> ArmState::getJointAnglesRadians()
{
	std::vector<double> jointAnglesRadians(JointAngles.size());		
	std::transform(JointAngles.begin(),JointAngles.end(),jointAnglesRadians.begin(),AS5048::stepsToRadians);
	return jointAnglesRadians;
}

vector<double> ArmState::getJointAngles()
{
	return JointAngles;
}

void ArmState::setJointAngles(vector<double> jointAngles)
{
	opSpaceStateComputed = false;
	this->JointAngles = jointAngles;
}

OpSpaceState ArmState::getOpSpaceState()
{
	if (!opSpaceStateComputed) calculateOpSpaceState();
	return computedOperationalSpaceState;
}

void ArmState::calculateOpSpaceState()
{
	Vector3d currentPosition; double r[9];		
	ComputeFk(getJointAnglesRadians().data(),currentPosition,r);
	Matrix3d currentRotation = Matrix3d::fromRowMajorArray(r);

	computedOperationalSpaceState = OpSpaceState(currentPosition,currentRotation);
	opSpaceStateComputed = true;
}




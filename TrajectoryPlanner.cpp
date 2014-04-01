#include "TrajectoryPlanner.hpp"

using namespace vmath;
using namespace std;

using namespace ikfast;

vector<OpSpaceState> TrajectoryPlanner::buildTrajectory(IKGoal goal)
{
	Vector3d targetPosition;
	Matrix3d targetRotation;

		
	Vector3d currentPosition;
	double r[9];	
	vector<double> lastAngles = currentArmState.getJointAnglesRadians();
	
	ComputeFk(lastAngles.data(),currentPosition,r);
	Matrix3d currentRotation = Matrix3d::fromRowMajorArray(r);
	
	switch (goal.action)
	{
		case IKGoal::Action::Position:
			targetPosition = goal.Position;
			targetRotation = currentRotation;
			break;
		case IKGoal::Action::PositionRotation:
			targetPosition = goal.Position;
			targetRotation = goal.Rotation;
			break;
		case IKGoal::Action::Rotation:
			targetPosition = currentPosition;
			targetRotation = goal.Rotation;
			break;
		case IKGoal::Action::Stop:
		default:
			throw std::logic_error("Cannot build steps for stop goal");
			break;
	}
	
	if (goal.relative)
	{
		targetPosition = currentPosition + targetPosition;
		//How do I added rotations?
	}

	Vector3d delta = targetPosition - currentPosition;
	
	if (delta.length() < 0.001)
	{
		throw std::runtime_error("Goal does not change position");
	}
		
	
	int numDivisions;
	switch (pathInterpolationMode)
	{
	case PathInterpolationMode::FixedStepCount:
		numDivisions = pathDivisionCount;
		break;
	case PathInterpolationMode::FixedStepDistance:
		numDivisions = std::round(delta.length()/pathInterpolationDistance);
		break;
	case PathInterpolationMode::SingleStep:
	default:
		numDivisions = 1;
		break;
	}

	vector<OpSpaceState> trajectory;

	for (int i=0;i<numDivisions;i++)
	{
		Vector3d stepPosition = currentPosition + delta*((double)i/(double)numDivisions);
		Matrix3d stepRotation = (i == 0) ? currentRotation : targetRotation;

		trajectory.push_back(OpSpaceState(stepPosition,stepRotation));
	}
	return trajectory;
}




void TrajectoryPlanner::setPathDivisions(int pathDivisionCount)
{
	this->pathDivisionCount = pathDivisionCount;
}

void TrajectoryPlanner::setPathInterpolationDistance(double pathInterpolationDistance)
{
	this->pathInterpolationDistance = pathInterpolationDistance;
}

void TrajectoryPlanner::setPathInterpolationMode(PathInterpolationMode pathInterpolationMode)
{
	this->pathInterpolationMode = pathInterpolationMode;
}
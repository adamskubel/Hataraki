#include "TrajectoryPlanner.hpp"

using namespace vmath;
using namespace std;
using namespace ikfast;

#include "AsyncLogger.hpp"


vector<OpSpaceState> TrajectoryPlanner::interpolateBetweenStates(OpSpaceState start, OpSpaceState end, int subdivisions)
{	
	vector<OpSpaceState> trajectory;

	Vector3d subDivisionOffset = (end.Position - start.Position) * (1.0/(double)subdivisions);

	for (int i=0;i<=subdivisions;i++)
	{
		Vector3d stepPosition = start.Position + subDivisionOffset*(double)i;
		Matrix3d stepRotation = (i == 0) ? start.Rotation : end.Rotation;

		trajectory.push_back(OpSpaceState(stepPosition,stepRotation));
		
//		cout << "Step: " << trajectory.back().toString() << endl;
	}
	return trajectory;
}

vector<OpSpaceState> TrajectoryPlanner::buildTrajectory(cJSON * trajectoryDefinition, bool relative)
{
	OpSpaceState initialState = cArmState.getOpSpaceState();

	vector<OpSpaceState> roughTrajectory;

	roughTrajectory.push_back(initialState);

	for (int i=0;i<cJSON_GetArraySize(trajectoryDefinition);i++)
	{
		cJSON * pathStep = cJSON_GetArrayItem(trajectoryDefinition,i);
		
		Vector3d position = Configuration::getVectorFromJSON(cJSON_GetObjectItem(pathStep,"Position"));	
		position /= 100.0; //cm -> m

		if (relative)
			position += initialState.Position;

		Vector3d eulerAngles = Configuration::getVectorFromJSON(cJSON_GetObjectItem(pathStep,"EulerRotation"));
		Matrix3d rotation = Matrix3d::createRotationAroundAxis(eulerAngles.x,eulerAngles.y,eulerAngles.z);
		
		roughTrajectory.push_back(OpSpaceState(position,rotation));
	}

	return roughTrajectory;
}

vector<OpSpaceState> TrajectoryPlanner::buildTrajectory(IKGoal goal)
{
	Vector3d targetPosition;
	Matrix3d targetRotation;
	
	OpSpaceState initialState = cArmState.getOpSpaceState();
	Vector3d currentPosition = initialState.Position;
	Matrix3d currentRotation = initialState.Rotation;
	
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
	
//	if (delta.length() < 0.000001)
//	{
//		throw std::runtime_error("Goal does not change position");
//	}
		
	
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
	
	return interpolateBetweenStates(initialState,OpSpaceState(targetPosition,targetRotation),numDivisions);
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


void TrajectoryPlanner::setArmState(ArmState newState)
{
	cArmState = newState;
}











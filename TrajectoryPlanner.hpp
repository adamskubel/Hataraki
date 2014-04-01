#ifndef HATARAKI_BASICMOTION_TRAJECTORY_PLANNER_HPP_
#define HATARAKI_BASICMOTION_TRAJECTORY_PLANNER_HPP_

#include <vector>

#include "MathUtils.hpp"
#include "OpSpaceState.hpp"

#include "IKFast.hpp"
#include "ArmState.hpp"

struct IKGoal {

	enum class Action {
		Position,
		PositionRotation,
		Rotation,
		Stop
	};

	vmath::Vector3d Position;
	vmath::Matrix3d Rotation;

	Action action;
	bool relative;
	
	IKGoal()
	{
		
	}
	
	IKGoal(vmath::Vector3d _Position, bool relative)
	{
		this->Position = _Position;
		this->relative = relative;
		
		action = Action::Position;
	}
	
	IKGoal(vmath::Vector3d _Position, vmath::Matrix3d _Rotation, bool relative)
	{
		this->Position = _Position;
		this->Rotation = _Rotation;
		this->relative = relative;

		action = Action::PositionRotation;
	}

//	IKGoal(vmath::Matrix3d _Rotation, bool relative)
//	{
//		this->Rotation = _Rotation;
//		this->relative = relative;
//
//		action = Action::Rotation;
//	}

public:
	static IKGoal stopGoal() {
		IKGoal g;
		g.action = Action::Stop;
		return g;
	}
	
	std::string toString()
	{
		std::stringstream ss;
		if (relative) ss << "[R]";
		ss << "Target = " << Position.toString();
		return ss.str();
	}
	
};


enum class PathInterpolationMode {
	
	SingleStep,
	FixedStepCount,
	FixedStepDistance
};


class TrajectoryPlanner {

private:	
	double pathInterpolationDistance;	
	int pathDivisionCount;
	PathInterpolationMode pathInterpolationMode;

	ArmState currentArmState;

public:
	
	void setPathDivisions(int divisionCount);
	void setPathInterpolationDistance(double pathInterpolationDistance);
	void setPathInterpolationMode(PathInterpolationMode pathInterpolationMode);

	std::vector<OpSpaceState> buildTrajectory(IKGoal goal);

	void setState(ArmState state);

};


#endif
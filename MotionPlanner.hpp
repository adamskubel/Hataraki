#ifndef HATARAKI_BASICMOTION_IKPLANNER_HPP_
#define HATARAKI_BASICMOTION_IKPLANNER_HPP_

#include <vector>
#include <memory>
#include <iostream>

#include "MotionPlan.hpp"
#include "PredictiveJointController.hpp"
#include "KinematicSolver.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"
#include "ikfast.h"
	


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


struct Step {
	
	std::vector<double> Positions, Velocities, Accelerations, Jerks;
	
	
	double Time;
	double TimeOffset; //Offset from previous interval

	Step(int numChannels) {

		this->TimeOffset = 0.0;
		this->Time = -1;
		for (int i=0;i<numChannels;i++)
		{
			Positions.push_back(0);
			Accelerations.push_back(0);
			Velocities.push_back(0);
			Jerks.push_back(0);
		}
	}
		
	Step(std::vector<double>  _Positions) {
		this->Positions = _Positions;
		this->TimeOffset = 0.0;
		this->Time = -1;
		for (int i=0;i<Positions.size();i++)
		{
			Accelerations.push_back(0);
			Velocities.push_back(0);
			Jerks.push_back(0);
		}
	}
};

struct StepMotionPlan
{
	std::vector<MotionInterval> intervals;
	
	StepMotionPlan()
	{
		this->isFinalVelocityEnforced = false;
		this->finalVelocity = 0;
	}
		
	StepMotionPlan(std::vector<MotionInterval> intervals)
	{
		this->isFinalVelocityEnforced = false;
		this->finalVelocity = 0;
		this->intervals = intervals;
	}
		
public:
	double finalVelocity;
	bool isFinalVelocityEnforced;
		
public:
	void setFinalVelocity(double finalVelocity)
	{
		this->finalVelocity = finalVelocity;
		isFinalVelocityEnforced = true;
	}
	
	double getMotionPlanFinalVelocity()
	{
		if (intervals.empty()) return 0;
		else return intervals.back().endSpeed;
	}
};

enum PathInterpolationMode {
	
	SingleStep,
	FixedStepCount,
	FixedStepDistance
};

class MotionPlanner {
	
private:
	std::vector<PredictiveJointController*> joints;
	
	double pathInterpolationDistance;	
	int pathDivisionCount;
	PathInterpolationMode pathInterpolationMode;
		
	std::vector<std::shared_ptr<MotionPlan> > compileStepMotionPlans(std::vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps);

	//IK stuff
	bool getEasiestSolution(const double * currentAngles, vmath::Vector3d targetPosition, vmath::Matrix3d targetRotation, double * result);
	bool checkSolutionValid(const double * solution);
	double calculateMotionEffort(const double * solution0, const double * solution1);
		
	std::vector<double> getJointAnglesRadians();

	std::vector<std::shared_ptr<MotionPlan> > buildPlanForSmoothStop();
	
	void validatePlan(IKGoal goal, std::vector<std::shared_ptr<MotionPlan> > plan);
	
	static std::shared_ptr<MotionPlan> buildMotionPlan(const double startPosition,const double endPosition, const double totalTime, const double approachVelocity, const double maxAccel);
	std::vector<std::shared_ptr<MotionPlan> > createClosedSolutionMotionPlanFromSteps(std::vector<Step> & steps);
	
	std::vector<std::shared_ptr<MotionPlan> > buildPlan(std::vector<Step> & steps);
	void calculateStep(std::vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps, int stepNumber);
	std::vector<Step> buildMotionSteps(IKGoal goal);
	
public:
	MotionPlanner(std::vector<PredictiveJointController*> joints);
	
	void setPathDivisions(int divisionCount);
	void setPathInterpolationDistance(double pathInterpolationDistance);
	void setPathInterpolationMode(PathInterpolationMode pathInterpolationMode);
		
	std::vector<std::shared_ptr<MotionPlan> > buildPlan(IKGoal goal);
	std::shared_ptr<MotionPlan> buildOptimalMotionPlan(int jointIndex, const double targetAngle);
	
};


#endif

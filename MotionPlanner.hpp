#ifndef HATARAKI_BASICMOTION_IKPLANNER_HPP_
#define HATARAKI_BASICMOTION_IKPLANNER_HPP_

#include <vector>
#include <memory>
#include <iostream>

#include "MotionPlan.hpp"
#include "PredictiveJointController.hpp"
#include "KinematicSolver.hpp"
#include "MathUtils.hpp"
#include "OpSpaceState.hpp"
#include "ArmState.hpp"

#include "IKFast.hpp"
	



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

class MotionPlanner {
	
private:
	std::vector<PredictiveJointController*> joints;
	ArmState cArmState;
	
		
	std::vector<std::shared_ptr<MotionPlan> > compileStepMotionPlans(std::vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps);

	//IK stuff
	bool getEasiestSolution(const double * currentAngles, vmath::Vector3d targetPosition, vmath::Matrix3d targetRotation, std::vector<double> & result);
	bool checkSolutionValid(const double * solution);
	double calculateMotionEffort(const double * solution0, const double * solution1);
		

	std::vector<std::shared_ptr<MotionPlan> > buildPlanForSmoothStop();
	
	void validatePlan(std::vector<OpSpaceState> goal, std::vector<std::shared_ptr<MotionPlan> > plan);
	
	static std::shared_ptr<MotionPlan> buildMotionPlan(const double startPosition,const double endPosition, const double totalTime, const double approachVelocity, const double maxAccel);
	std::vector<std::shared_ptr<MotionPlan> > createClosedSolutionMotionPlanFromSteps(std::vector<Step> & steps, bool coastPhase);
	
	std::vector<std::shared_ptr<MotionPlan> > buildPlan(std::vector<Step> & steps);
	void calculateStep(std::vector<StepMotionPlan> * stepPlans, std::vector<Step> & steps, int stepNumber);
	std::vector<Step> buildMotionSteps(std::vector<OpSpaceState> goal);
	
public:
	MotionPlanner(std::vector<PredictiveJointController*> joints);
	
		
	std::vector<std::shared_ptr<MotionPlan> > buildPlan(std::vector<OpSpaceState> trajectory);
	std::shared_ptr<MotionPlan> buildOptimalMotionPlan(int jointIndex, const double targetAngle);

	void setArmState(ArmState newArmState);
	
};


#endif

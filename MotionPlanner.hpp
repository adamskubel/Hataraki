#ifndef HATARAKI_BASICMOTION_IKPLANNER_HPP_
#define HATARAKI_BASICMOTION_IKPLANNER_HPP_

#include <vector>
#include <memory>
#include <iostream>

#include "MotionPlan.hpp"
#include "PathPlanner.hpp"
#include "PredictiveJointController.hpp"
#include "KinematicSolver.hpp"

#define VMATH_NAMESPACE vmath
#include "vmath.h"
#include "ikfast.h"

class MotionPlanner {
		
private:
	PathPlanner * pathPlanner;
	std::vector<PredictiveJointController*> joints;
	
	double interpolationDistance;
	int pathDivisionCount;
	double firstStepTime, lastStepTime;
	
	
	bool getEasiestSolution(const double * currentAngles, vmath::Vector3d targetPosition, vmath::Matrix3d targetRotation, double * result);
	bool checkSolutionValid(const double * solution);
	double calculateMotionEffort(const double * solution0, const double * solution1);
	

	
public:
	MotionPlanner(std::vector<PredictiveJointController*> joints);
	
	std::vector<Step> realizeSteps(std::vector<Step> & steps);
	std::vector<Step> buildMotionSteps(std::vector<double> initialAngles, vmath::Vector3d position, vmath::Matrix3d targetRotation);
	
	static std::shared_ptr<MotionPlan> buildMotionPlan(const double startPosition,const double endPosition, const double totalTime, const double approachVelocity, const double maxAccel);

	static std::vector<std::shared_ptr<MotionPlan> > convertStepsToPlans(std::vector<Step> & steps, std::vector<double> initialAngles);

	void setPathDivisions(int divisionCount);
	
};


#endif

#ifndef HATARAKI_BASICMOTION_KINEMATIC_PLANNING_HPP_
#define HATARAKI_BASICMOTION_KINEMATIC_PLANNING_HPP_

#include "MathUtils.hpp"
#include <cmath>

#define MathDebug true

struct PlanSolution {
	
	double i_dTotal, i_tTotal, i_v0, i_v2, i_d3;
	
	double t0,t1,t2,t3;
	double travelVelocity;
	double accel0,accel1;
	bool valid;
};

class KinematicSolver {
	
public:
	static double optimalSpeed(const double a0, const double d3, const double dTotal, const double v0, const double v2, const double maxSpeed, double & speed);
	static void calculatePlan(double absAccel, double d3, double tTotal, double dTotal, double v0, double v2, PlanSolution & result);
	
	static double optimalSpeed2Part(const double a0, const double dTotal, const double v0, const double maxSpeed, double & speed);
	static void calculatePlan2Part(double absAccel, double tTotal, double dTotal, double v0, PlanSolution & result);
	
	static void validateSolution(PlanSolution & sol);
	
};


#endif
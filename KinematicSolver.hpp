#ifndef HATARAKI_BASICMOTION_KINEMATIC_PLANNING_HPP_
#define HATARAKI_BASICMOTION_KINEMATIC_PLANNING_HPP_

#include "MathUtils.hpp"
#include <cmath>

#define MathDebug true

struct PlanSolution {
	
	double i_dTotal, i_tTotal, i_v0, i_v2, i_d3;
	
	double t0,t1,t2,t3;
	double v1;
	double accel0,accel1;
	bool valid;
};

class KinematicSolver {
	
public:
	static double optimalSpeed(const double a0, const double d3, const double d, const double v0, const double v2, const double maxSpeed, double & speed);
	static void calculatePlan(double absAccel, double d3, double tTotal, double d, double v0, double v2, PlanSolution & result);
		
	static double optimalSpeed2Part(const double a0, const double d, const double v0, const double maxSpeed, double & speed);

	static bool		twoPart_checkSolutionExists(double aMax, double v0, double delta, double time);	
	static double	twoPart_minimumTime(const double accel, const double d, const double v0, const double maxSpeed);
	static double	twoPart_maxInitialVelocity(const double aMax, const double d, const double tTotal);
	static void		twoPart_calculate(double absAccel, double tTotal, double d, double v0, PlanSolution & result);
			
	static bool		threePart_checkTimeInvariantSolutionExists(double aMax, double initialVelocity, double finalVelocity, double delta);
	static bool		threePart_checkSolutionExists(double aMax, double initialVelocity, double finalVelocity, double delta, double time);
	static double	threePart_minimumTime(const double aMax, const double vMax, const double distance, const double v0, const double v1);
	static double	threePart_maxTimeInvariantInitialVelocity(const double aMax, const double vF, const double d);
	static double	threePart_maxInitialVelocity(const double aMax, const double vF, const double d, const double tTotal);
	static void		threePart_calculate(const double aMax, const double d, const double v0, const double vF, const double tTotal, PlanSolution & result);
	
	static void validateSolution(PlanSolution & sol);
	
};


#endif
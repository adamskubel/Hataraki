#ifndef HATARAKI_BASICMOTION_KINEMATIC_PLANNING_HPP_
#define HATARAKI_BASICMOTION_KINEMATIC_PLANNING_HPP_

#include "MathUtils.hpp"
#include <cmath>

#define MathDebug true



struct PlanSolution {

	enum SolutionStatus 
	{
		NoSolution,
		AdjustedSolution,
		OriginalSolution
	};

	struct Input
	{
		double v0,vF,d,t,v3;
	};
	Input input;
		
	double t0,t1,t2,t3,v1;
	double a0,a1;


	SolutionStatus status;

	struct Adjusted
	{
		double v0_min, v0_max;
	};
	Adjusted adjusted;

public:
	void setResult(double v1, double t0, double t1, double t2)
	{
		this->v1 = v1;
		this->t0 = t0;
		this->t1 = t1;
		this->t2 = t2;
	}

	void setResult(double v1, double t0, double t1, double t2, double t3)
	{
		this->v1 = v1;
		this->t0 = t0;
		this->t1 = t1;
		this->t2 = t2;
		this->t3 = t3;
	}
};

class KinematicSolver {
	
public:
	static double	fourPart_minimumTime(const double a0, const double d3, const double d, const double v0, const double v2, const double maxSpeed, double & speed);
	static void		fourPart_calculate(double absAccel, double d3, double tTotal, double d, double v0, double v2, PlanSolution & result);
		
	static double	optimalSpeed2Part(const double a0, const double d, const double v0, const double maxSpeed, double & speed);

	static bool		twoPart_checkSolutionExists(double aMax, double v0, double delta, double time);	
	static double	twoPart_minimumTime(const double accel, const double d, const double v0, const double maxSpeed);
	static double	twoPart_maxInitialVelocity(const double aMax, const double d, const double tTotal);
	static bool		twoPart_attemptSolution(const double aMax, const double v0, const double d, const double t, double & v0_new);
	static void		twoPart_calculate(double absAccel, double v0, double d, double tTotal, PlanSolution & result);
			
	static bool		threePart_checkTimeInvariantSolutionExists(double aMax, double initialVelocity, double finalVelocity, double delta);
	static double	threePart_minimumTime(const double aMax, const double vMax, const double v0, const double vF, const double d);
	static double	threePart_maxTimeInvariantInitialVelocity(const double aMax, const double vF, const double d);
	static bool		threePart_attemptSolution(const double aMax, const double d, const double v0, const double vF, const double t, double & v0_new);
	static void		threePart_calculate(const double aMax, const double d, const double v0, const double vF, const double tTotal, PlanSolution & result);
	
	static void validateSolution(PlanSolution & sol);
	
};


#endif
#include "KinematicSolver.hpp"
#include <iostream>

using namespace std;

void KinematicSolver::validateSolution(PlanSolution & sol)
{
	////if (!sol.valid) cout << "Solution is invalid" << endl;
	//if ((sol.t0 < 0 || std::isnan(sol.t0)) || (sol.t1 < 0 || std::isnan(sol.t1)) || (sol.t2 < 0 || std::isnan(sol.t2)) || (sol.t3 < 0 || std::isnan(sol.t3)) || std::isnan(sol.v1))
	//{
	//	cout << "Sol.T0 = " << sol.t0 << "\t";
	//	cout << "Sol.T1 = " << sol.t1 << "\t";
	//	cout << "Sol.T2 = " << sol.t2 << "\t";
	//	cout << "Sol.T3 = " << sol.t3 << "\t";
	//	cout << "Sol.A0 = " << sol.a0 << "\t";
	//	cout << "Sol.A1 = " << sol.a1 << "\t";
	//	cout << "Sol.V1 = " << sol.v1 << endl;
	//	
	//	//cout << "Input: T_Total = " << sol.i_tTotal << "\tD_Total = " << sol.i_dTotal << "\tV0 = " << sol.i_v0 << "\tD3 = " << sol.i_d3 << "\tV2 = " << sol.i_v2 << endl;
	//}
} 

// ****** Two part problem **********

double KinematicSolver::twoPart_maxInitialVelocity(const double aMax, const double d, const double tTotal)
{
	// Solving the equation:
	//tD*v0 - 0.5A*tD^2 == D_target, solve for v0
	// v0 == D_target/tD + 0.5A*tD
	double a0 = -std::abs(aMax)*sgn(d);
	return d/tTotal + a0*0.5*tTotal;
}

double KinematicSolver::twoPart_minimumTime(const double aMax, const double vMax, const double d, const double v0)
{
	double a0 = aMax * sgn(d);
	
	double v1 = sgn(d)*sqrt(a0*d*2.0+v0*v0);
	
	if (std::isnan(v1))
	{
		throw std::runtime_error("Complex result in two-part minimization");
		//a0 = -a0;
		//v1 = sgn(d)*sqrt(a0*d*2.0+v0*v0);
	}
	
	if (std::abs(v1) > std::abs(vMax))
		v1 = std::abs(vMax)*sgn(v1);
	
	return (a0*d-v0*v1+(v0*v0)*(1.0/2.0)+(v1*v1)*(1.0/2.0))/(a0*v1);
}

bool KinematicSolver::twoPart_attemptSolution(const double aMax, const double v0, const double d, const double t, double & v0_new)
{
	PlanSolution sol;
	twoPart_calculate(aMax,v0,d,t,sol);

	if (sol.status == PlanSolution::SolutionStatus::AdjustedSolution)
	{
		v0_new = sol.adjusted.v0_max;
		return false;
	}
	else if (sol.status == PlanSolution::SolutionStatus::OriginalSolution)
	{
		return true;
	}
	else
	{
		throw std::logic_error("Two part problem should always have a possible solution.");
	}
}


void KinematicSolver::twoPart_calculate(double aMax,double v0, double d, double tTotal, PlanSolution & sol)
{
	double v1,t0,t1,a0;

	if (abs(v0 * tTotal) > abs(d))
		a0 = aMax * -sgn(d);
	else
		a0 = aMax * sgn(d);

	sol.status = PlanSolution::SolutionStatus::NoSolution;
	
	double t0_sign = -(sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
	
	if (std::isnan(t0_sign))
	{
		a0 = -std::abs(aMax)*sgn(d);
		double v0_max = d/tTotal + a0*0.5*tTotal;
		sol.adjusted.v0_max = v0_max;
		sol.status = PlanSolution::AdjustedSolution;
		return;
	}
	
	if (a0 < 0)
	{
		v1 = v0+sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = (sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal)/a0;
		t1 = -sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
		sol.status = PlanSolution::OriginalSolution;
	}
	else
	{
		v1 = v0-sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = -(sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
		t1 = sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
		sol.status = PlanSolution::OriginalSolution;
	}
	
	sol.a0 = a0;
	sol.setResult(v1,t0,t1,0);
	
	if (MathDebug) validateSolution(sol);
}

// ********** Three part problem *************

bool KinematicSolver::threePart_checkTimeInvariantSolutionExists(double aMax, double initialVelocity, double finalVelocity, double delta)
{
	double minTime = std::abs((finalVelocity - initialVelocity)/aMax);
	double minDist = initialVelocity*minTime + 0.5*sgn(delta)*aMax*std::pow(minTime,2);

	if (sgn(minDist) == sgn(delta) && std::abs(minDist) > std::abs(delta))
		return false;
	return true;
}

double KinematicSolver::threePart_minimumTime(const double aMax, const double vMax, const double v0, const double v2, const double d)
{
	double a0 = aMax * sgn(d);

	double v1 = sgn(d)*sqrt(2.0)*sqrt(a0*d*2.0+v0*v0+v2*v2)*(1.0/2.0);
	
	if (abs(v1) > vMax)
		v1 = sgn(v1)*vMax;
	
	double minTime = (a0*d-v0*v1-v1*v2+(v0*v0)*(1.0/2.0)+v1*v1+(v2*v2)*(1.0/2.0))/(a0*v1);

	return minTime;
}

bool KinematicSolver::threePart_attemptSolution(const double aMax, const double v0, const double vF, const double d,  const double t, double & v0_new)
{
	PlanSolution sol;
	threePart_calculate(aMax,v0,vF,d,t,sol);

	if (sol.status == PlanSolution::SolutionStatus::NoSolution)
	{
		throw std::runtime_error("No solution is possible");
	}
	else if (sol.status == PlanSolution::SolutionStatus::AdjustedSolution)
	{
		if (v0 > sol.adjusted.v0_max)
			v0_new = sol.adjusted.v0_max;
		else if (v0 < sol.adjusted.v0_min)
			v0_new = sol.adjusted.v0_min;
		else
			throw std::logic_error("Attempted to adjust initial velocity, but it is already within the acceptable range.");

		return false;
	}
	return true;
}

void KinematicSolver::threePart_calculate(double aMax, double v0, double vF, double d, double t, PlanSolution & sol)
{
	double v1,t0,t1,t2;

	sol.status = PlanSolution::SolutionStatus::NoSolution;
	int form = 1;
	int dir = sgn(d);

	//Solution 1 
	// U shape (decel - travel - accel) AKA CONCAVE
	// test: a0 negative, a1 positive, if v1 is negative then test fails and no valid solution exists 
	// If V1 is complex, then the time interval is too short for this solution. (go to 2)
	// If T2 is negative, S3 might work?
	// If t2 and t1 are zero, then the solution is a straight line

	if (form == 1)
	{
		double a0 = -dir*aMax, a1= dir*aMax;

		threePartEval(a0,a1,d,t,v0,vF,v1,t0,t1,t2);
	
		if (std::isnan(v1) || (t0 < 0 || t2 < 0))
		{
			form = 2;
		}
		else if (t2 < 0 && abs(v1) > abs(vF) && sgn(v1) == sgn(vF))
		{
			form = 2;
		}
		else if (abs(v1) > 0 && sgn(v1) != dir)
		{
			sol.status = PlanSolution::AdjustedSolution;
			double concave_v0_max = sqrt(a0*d*-2.0-vF*vF);
			sol.adjusted.v0_max = concave_v0_max;
			sol.adjusted.v0_min = 0;
			
			if (isnan(concave_v0_max))
			{
				throw logic_error("NAAN");
			}
			
			sol.setResult(v1,t0,t1,t2);
		}
		else
		{
			sol.status = PlanSolution::SolutionStatus::OriginalSolution;
			sol.setResult(v1,t0,t1,t2);
			sol.assertValidResult();
		}
	}
	
	//Solution 2 (fastest) AKA CONVEX
	// n shape (accel - travel - decel)
	// test: a0 positive, a1 negative. If t0 and/or t2 are negative, then the time is too long for this solution (go to 3)
	// If v1 is complex, then the time is too short for this solution (and thus no solution is possible)
	
	if (form == 2)
	{
		double a0 = dir*aMax, a1 = -dir*aMax;
		threePartEval(a0,a1,d,t,v0,vF,v1,t0,t1,t2);
	
		if (isnan(v1))
		{
			sol.status = PlanSolution::AdjustedSolution;
			double convex_v0_min = vF+a0*t+sqrt(2.0)*sqrt(a0*(d*-2.0+t*vF*2.0+a0*(t*t)));
			double convex_v0_max = vF+sqrt(a0*(d*-2.0+t*vF*2.0+a0*(t*t)))+a0*t;
			double convex_v0_max_2= vF-sqrt(a0*(d*-2.0+t*vF*2.0+a0*(t*t)))+a0*t;
			
			sol.adjusted.v0_min = convex_v0_min;
			sol.adjusted.v0_max = convex_v0_max_2;
			
			if (isnan(convex_v0_max_2) || isnan(convex_v0_min))
			{
				throw logic_error("NAAN-2");
			}
		}
		else if (t0 < 0 || t2 < 0)
		{
			form = 3;
		}
		else
		{
			sol.status = PlanSolution::SolutionStatus::OriginalSolution;
			sol.setResult(v1,t0,t1,t2);
			sol.assertValidResult();
		}
	}

	//Solution 3
	// \__
	//    \
	// (decel - travel - decel) or (accel - travel - accel)
	// try if S1 is complex (too slow) and S2 is negative (too fast)
	// test: a0 = a1 = aMax * sgn(vF-v0)
	// If t2 is negative, time is too slow
	// If t0 or t1 is negative, time is too fast
	// But, if we obey the above preconditions, this solution should always work
	
	if (form == 3)
	{
		double a0,a1;
		a0 = a1 = sgn(vF-v0) * aMax;
		threePartEval(a0,a1,d,t,v0,vF,v1,t0,t1,t2);
		//done unless I fucked up
		sol.setResult(v1,t0,t1,t2);
		
		sol.assertValidResult();
		sol.status = PlanSolution::SolutionStatus::OriginalSolution;
	}
}

void KinematicSolver::threePartEval(double a0, double a1, double dTotal, double tTotal, double v0, double v2, double & v1, double & t0, double & t1, double & t2)
{
	if (a0 == a1)
	{
		v1 = (a0*dTotal*2.0+v0*v0-v2*v2)/(v0*2.0-v2*2.0+a0*tTotal*2.0);
		t0 = (a0*dTotal+v0*v2-(v0*v0)*(1.0/2.0)-(v2*v2)*(1.0/2.0)-a0*tTotal*v0)/(a0*(v0-v2+a0*tTotal));
		t1 = (v0-v2+a0*tTotal)/a0;
		t2 = (-a0*dTotal+v0*v2-(v0*v0)*(1.0/2.0)-(v2*v2)*(1.0/2.0)+a0*tTotal*v2)/(a0*(v0-v2+a0*tTotal));
	}	
	else 
	{
		v1 = -(a1*v0-a0*v2+sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal)/(a0-a1);
		t0 = -(v0+(a1*v0-a0*v2+sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal)/(a0-a1))/a0;
		t1 = (a1*v0-a0*v2-(a0*(a1*v0-a0*v2+sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal))/(a0-a1)+
			  (a1*(a1*v0-a0*v2+sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal))/(a0-a1)+a0*a1*tTotal)/(a0*a1);
		t2 = (v2+(a1*v0-a0*v2+sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal)/(a0-a1))/a1;
	}
	//else
	//{
	//	v1 = -(a1*v0-a0*v2-sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal)/(a0-a1);
	//	t0 = -(v0+(a1*v0-a0*v2-sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal)/(a0-a1))/a0;
	//	t1 = (a1*v0-a0*v2+a0*a1*tTotal-(a0*(a1*v0-a0*v2-sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))
	//										+a0*a1*tTotal))/(a0-a1)+(a1*(a1*v0-a0*v2-sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal))/(a0-a1))/(a0*a1);
	//	t2 = (v2+(a1*v0-a0*v2-sqrt(a0*a1*(a0*dTotal*2.0-a1*dTotal*2.0-v0*v2*2.0+v0*v0+v2*v2+a1*tTotal*v0*2.0-a0*tTotal*v2*2.0+a0*a1*(tTotal*tTotal)))+a0*a1*tTotal)/(a0-a1))/a1;
	//}
}

double KinematicSolver::fourPart_minimumTime(const double accel, const double d3, const double d, const double v0, const double v2, const double maxSpeed, double & v1)
{
	double a0 = std::abs(accel);
	double direction = sgn(d);
	
	v1 = direction*sqrt(2.0)*sqrt(a0*d3*-2.0+a0*d*2.0+v0*v0+v2*v2)*(1.0/2.0);
	
	if (std::isnan(v1))
	{
		a0 = -a0;
		v1 = direction*sqrt(2.0)*sqrt(a0*d3*-2.0+a0*d*2.0+v0*v0+v2*v2)*(1.0/2.0);
	}
	
	if (std::abs(v1) > std::abs(maxSpeed))
		v1 = std::abs(maxSpeed)*sgn(v1);
	
	return ((v0*v0)*v2*(1.0/2.0)-v1*(v2*v2)+(v1*v1)*v2+(v2*v2*v2)*(1.0/2.0)+a0*d3*v1-a0*d3*v2+a0*d*v2-v0*v1*v2)/(a0*v1*v2);
}

void KinematicSolver::fourPart_calculate(double absAccel, double d3, double tTotal, double d, double v0, double v2, PlanSolution & result)
{
	double v1,t0,t1,t2,t3;
	result.status = PlanSolution::SolutionStatus::NoSolution;
	
	double a0,a1;
	
	if (d > 0)
	{
		a0 = std::abs(absAccel);
		a1 = -std::abs(absAccel);
	}
	else
	{
		a0 = -std::abs(absAccel);
		a1 = std::abs(absAccel);
	}
	
	//result.i_d3 = d3;
	//result.i_dTotal = d;
	//result.i_v0 = v0;
	//result.i_v2 = v2;
	//result.i_tTotal = tTotal;
	
		
	double t0_sign,t2_sign;

	double t1_sign = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2+(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
	(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
	*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2)-(a1*v2*
	(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*
	(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);

	if (t1_sign >= 0)
	{
		t0_sign = -(v0+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*
		(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
		
		t2_sign = (v2+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)
	   *2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
	}
	else
	{
		t0_sign = -(v0-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)
				*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
		
		t2_sign = (v2-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)
			   *2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
	}
	
	//Case 1: Both T0 and T2 are positive = good
	//Case 2: T0 is negative -> Flip sign of A0 and use EQ 2
	//Case 3: T2 is negative -> Flip sign of A1 and use EQ 2
	//Case 4: Both T0 and T2 are negative -> flip sign of both and use EQ 1
	//Case 5: Either T0 or T2 have non-real components (NaN). Return invalid result.
	
	
	if (std::isnan(t0_sign) || std::isnan(t2_sign)) return;
	if (t0_sign < 0) a0 = -a0;
	if (t2_sign < 0) a1 = -a1;
	
	if (a0 == a1)
	{
		v1 = -((v0*v0)*v2-v2*v2*v2-a0*d3*v2*2.0+a0*d*v2*2.0)/(a0*d3*2.0-v0*v2*2.0+(v2*v2)*2.0-a0*tTotal*v2*2.0);
		t0 = (-v0*(v2*v2)+(v0*v0)*v2*(1.0/2.0)+(v2*v2*v2)*(1.0/2.0)-a0*d3*v0+a0*d3*v2-a0*d*v2+a0*tTotal*v0*v2)/(a0*(a0*d3-v0*v2+v2*v2-a0*tTotal*v2));
		t1 = -(a0*d3-v0*v2+v2*v2-a0*tTotal*v2)/(a0*v2);
		t2 = (-v0*(v2*v2)+(v0*v0)*v2*(1.0/2.0)+(v2*v2*v2)*(1.0/2.0)-a0*tTotal*(v2*v2)+a0*d*v2)/(a0*(a0*d3-v0*v2+v2*v2-a0*tTotal*v2));
		t3 = d3/v2;
	}
	else
	{
		t1 = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2+(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
		(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
		*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2)-(a1*v2*
		(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*
		(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);
		
		if (t1 >= 0)
		{
			
			v1 = -(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)
							   *2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2);
			
			t0 = -(v0+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*
								   (v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
			
			t2 = (v2+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)
								  *2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
			
			t3 = d3/v2;
		}
		else //other solution
		{
			v1 = (sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2);
			
			t0 = -(v0-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
			
			t1 = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2-(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
				(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
				*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2))/(a0*v2-a1*v2)+
				(a1*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*
				(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);
			
			t2 = (v2-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*d*(v2*v2)*2.0-a1*d*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
			
			t3 = d3/v2;
		}
	}
	
	result.t0 = t0;
	result.t1 = t1;
	result.t2 = t2;
	result.t3 = t3;
	result.a0 = a0;
	result.a1 = a1;
	result.v1 = v1;
	result.status = PlanSolution::SolutionStatus::OriginalSolution;
	
	if (MathDebug) validateSolution(result);
}


/*



bool KinematicSolver::twoPart_checkSolutionExists(const double aMax, const double v0, const double d, const double t)
{
	if (sgn(v0) != 0 && sgn(v0) != sgn(d))
		return false;

	if (abs(v0 * t) > abs(d)) //possible overshoot
	{
		double a0 = -abs(aMax)*sgn(d);

		//Solution exists if sign changes
		return sgn(v0*t - a0*0.5*pow(t,2)) != sgn(d);
	}
	else //possible undershoot
	{
		double a0 = abs(aMax)*sgn(d);
		return abs(v0*t + a0*0.5*pow(t,2)) >= abs(d);
	}
}

double KinematicSolver::threePart_maxInitialVelocity(const double aMax, const double vF, const double d, const double t)
{
	//Solution 1 :
	//TYPE A - A valid solution exists for all speed between 0 and V1(v0) == 0
	//In this case, too slow is impossible. So just choose the fastest speed (better to choose a speed that results in smoother motion)
	//double concave_v0_max = sqrt(a0*dTotal*-2.0-vF*vF);


	//Solution 2 - The range of possible input velocities is large. 
	// It can be any value between the point at which t0/t1/t2 become real, and the point at which they become negative (x-intersect).	
	//TYPE B - A valid solution exists for a narrow range of speeds. Current speed can be too slow or too fast.

	//double a0 = aMax;

	//double convex_v0_min = vF+a0*t+sqrt(2.0)*sqrt(a0*(d*-2.0+t*vF*2.0+a0*(t*t)));

	//double convex_v0_max = vF+sqrt(a0*(d*-2.0+t*vF*2.0+a0*(t*t)))+a0*t;
	//double convex_v0_max_2= vF-sqrt(a0*(d*-2.0+t*vF*2.0+a0*(t*t)))+a0*t;
	
	throw std::logic_error("Nope.");
	return 0;
}
*/

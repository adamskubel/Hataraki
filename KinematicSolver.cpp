#include "KinematicSolver.hpp"
#include <iostream>

using namespace std;

void KinematicSolver::validateSolution(PlanSolution & sol)
{
	if (!sol.valid) cout << "Solution is invalid" << endl;
	if ((sol.t0 < 0 || std::isnan(sol.t0)) || (sol.t1 < 0 || std::isnan(sol.t1)) || (sol.t2 < 0 || std::isnan(sol.t2)) || (sol.t3 < 0 || std::isnan(sol.t3)) || std::isnan(sol.v1))
	{
		cout << "Sol.T0 = " << sol.t0 << "\t";
		cout << "Sol.T1 = " << sol.t1 << "\t";
		cout << "Sol.T2 = " << sol.t2 << "\t";
		cout << "Sol.T3 = " << sol.t3 << "\t";
		cout << "Sol.A0 = " << sol.accel0 << "\t";
		cout << "Sol.A1 = " << sol.accel1 << "\t";
		cout << "Sol.V1 = " << sol.v1 << endl;
		
		cout << "Input: T_Total = " << sol.i_tTotal << "\tD_Total = " << sol.i_dTotal << "\tV0 = " << sol.i_v0 << "\tD3 = " << sol.i_d3 << "\tV2 = " << sol.i_v2 << endl;
	}
}

//tD*v0 - 0.5A*tD^2 == D_target, solve for v0
// v0 == D_target/tD + 0.5A*tD
double KinematicSolver::twoPart_maxInitialVelocity(const double aMax, const double d, const double tTotal)
{
	double a0 = -std::abs(aMax)*sgn(d);
	return d/tTotal + a0*0.5*tTotal;
}

bool KinematicSolver::twoPart_checkSolutionExists(const double aMax, const double v0, const double d, const double t)
{
	if (sgn(v0) != 0 && sgn(v0) != sgn(d))
		return false;

	if (abs(v0 * t) > abs(d)) //possible overshoot
	{
		double a0 = -abs(aMax)*sgn(d);

		//Solution exists if sign changes
		return sgn(v0*t - a0*0.5*t^2) != sgn(d);
	}
	else //possible undershoot
	{
		double a0 = abs(aMax)*sgn(d);
		return abs(v0*t + a0*0.5*pow(t,2)) >= abs(d);
	}
}

bool KinematicSolver::threePart_checkSolutionExists(double aMax, double v0, double vF, double d, double t)
{
	//No solution if initial velocity is in the wrong direction
	if (sgn(v0) != 0 && sgn(v0) != sgn(d))
		return false;

	if (abs(vF - v0)/abs(aMax) < t)
		return false;

	//Attempt calculation
	return true;
}

bool KinematicSolver::threePart_checkTimeInvariantSolutionExists(double aMax, double initialVelocity, double finalVelocity, double delta)
{
	double minTime = std::abs((finalVelocity - initialVelocity)/aMax);
	double minDist = initialVelocity*minTime + 0.5*sgn(delta)*aMax*std::pow(minTime,2);

	if (sgn(minDist) == sgn(delta) && std::abs(minDist) > std::abs(delta))
		return false;
	return true;
}

void KinematicSolver::threePart_calculate(double aMax, double delta, double v0, double vF, double time, PlanSolution & sol)
{
	//Solution 1 
	// U shape (decel - travel - accel)
	// test: a0 negative, a1 positive, if v1 is negative then test fails and no valid solution exists
	// If V1 is complex, then the time interval is too short for this solution.
	// If t2 and t1 are zero, then the solution is a straight line
	
	//Solution 2 (fastest)
	// n shape (accel - travel - decel)
	// test: a0 positive, a1 negative. If t0 and/or t2 are negative, then the time is too slow for this solution
	// If v1 is complex, then the time is too short for this solution (and thus no solution is possible)

	//Solution 3
	// \__
	//    \
	// (decel - travel - decel) or (accel - travel - accel)
	// try if S1 is complex (too slow) and S2 is negative (too fast)
	// test: a0 = a1 = aMax * sgn(vF-v0)
	// If t2 is negative, time is too slow
	// If t0 or t1 is negative, time is too fast
	// But, if we obey the above preconditions, this solution should always work


}

double KinematicSolver::threePart_maxTimeInvariantInitialVelocity(const double aMax, const double vF, const double d)
{
	//The optimal solution for time invariant problem is always a straight line
	
}

double KinematicSolver::threePart_maxInitialVelocity(const double aMax, const double vF, const double d, const double t)
{
	//THIS IS HARD

	//NEXT STEP: Plot T0,T1,T2 with respect to V0. ALL ON THE SAME GRAPH!!!
}


double KinematicSolver::optimalSpeed(const double accel, const double d3, const double d, const double v0, const double v2, const double maxSpeed, double & v1)
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
	//else
	//v1 *= 0.99;
	
	return ((v0*v0)*v2*(1.0/2.0)-v1*(v2*v2)+(v1*v1)*v2+(v2*v2*v2)*(1.0/2.0)+a0*d3*v1-a0*d3*v2+a0*d*v2-v0*v1*v2)/(a0*v1*v2);
}

double KinematicSolver::twoPart_minimumTime(const double aMax, const double vMax, const double d, const double v0)
{
	double s;
	return optimalSpeed2Part(aMax, d, v0, vMax, s);
}

double KinematicSolver::optimalSpeed2Part(const double accel, const double d, const double v0, const double maxSpeed, double & v1)
{
	double a0 = std::abs(accel);
	double direction = sgn(d);
	v1 = direction*sqrt(a0*d*2.0+v0*v0);
	
	if (std::isnan(v1))
	{
		a0 = -a0;
		v1 = direction*sqrt(a0*d*2.0+v0*v0);
	}
	
	if (std::abs(v1) > std::abs(maxSpeed))
		v1 = std::abs(maxSpeed)*sgn(v1);
	//else
	//v1 *= 0.99;
	
	return (a0*d-v0*v1+(v0*v0)*(1.0/2.0)+(v1*v1)*(1.0/2.0))/(a0*v1);
}


void KinematicSolver::twoPart_calculate(double absAccel, double tTotal, double d, double v0, PlanSolution & result)
{
	double v1,t0,t1;
	result.valid = false;
	result.t3 = 0;
	result.t2 = 0;
	result.accel1= 0;
	
	double a0 = std::abs(absAccel);
	
	result.i_dTotal = d;
	result.i_v0 = v0;
	result.i_tTotal = tTotal;
	
	double t0_sign = -(sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
	
	if (std::isnan(t0_sign)) return;
	
	if (t0_sign < 0)
	{
		a0 = -a0;
		v1 = v0+sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = (sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal)/a0;
		t1 = -sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
	}
	else
	{
		v1 = v0-sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = -(sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
		t1 = sqrt(a0*(d*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
	}
	
	result.t0 = t0;
	result.t1 = t1;
	result.accel0 = a0;
	result.v1 = v1;
	result.valid = true;
	
	if (MathDebug) validateSolution(result);
}

void KinematicSolver::calculatePlan(double absAccel, double d3, double tTotal, double d, double v0, double v2, PlanSolution & result)
{
	double v1,t0,t1,t2,t3;
	result.valid = false;
	
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
	
	result.i_d3 = d3;
	result.i_dTotal = d;
	result.i_v0 = v0;
	result.i_v2 = v2;
	result.i_tTotal = tTotal;
	
		
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
	result.accel0 = a0;
	result.accel1 = a1;
	result.v1 = v1;
	result.valid = true;
	
	if (MathDebug) validateSolution(result);
}

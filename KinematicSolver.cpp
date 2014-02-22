#include "KinematicSolver.hpp"
#include <iostream>

using namespace std;

void KinematicSolver::validateSolution(PlanSolution & sol)
{
	if (!sol.valid) cout << "Solution is invalid" << endl;
	if ((sol.t0 < 0 || std::isnan(sol.t0)) || (sol.t1 < 0 || std::isnan(sol.t1)) || (sol.t2 < 0 || std::isnan(sol.t2)) || (sol.t3 < 0 || std::isnan(sol.t3)) || std::isnan(sol.travelVelocity))
	{
		cout << "Sol.T0 = " << sol.t0 << "\t";
		cout << "Sol.T1 = " << sol.t1 << "\t";
		cout << "Sol.T2 = " << sol.t2 << "\t";
		cout << "Sol.T3 = " << sol.t3 << "\t";
		cout << "Sol.A0 = " << sol.accel0 << "\t";
		cout << "Sol.A1 = " << sol.accel1 << "\t";
		cout << "Sol.V1 = " << sol.travelVelocity << endl;
		
		cout << "Input: T_Total = " << sol.i_tTotal << "\tD_Total = " << sol.i_dTotal << "\tV0 = " << sol.i_v0 << "\tD3 = " << sol.i_d3 << "\tV2 = " << sol.i_v2 << endl;
	}
}


double KinematicSolver::optimalSpeed(const double accel, const double d3, const double dTotal, const double v0, const double v2, const double maxSpeed, double & v1)
{
	double a0 = std::abs(accel);
	double direction = MathUtils::sgn<double>(dTotal);
	
	v1 = direction*sqrt(2.0)*sqrt(a0*d3*-2.0+a0*dTotal*2.0+v0*v0+v2*v2)*(1.0/2.0);
	
	if (std::isnan(v1))
	{
		a0 = -a0;
		v1 = direction*sqrt(2.0)*sqrt(a0*d3*-2.0+a0*dTotal*2.0+v0*v0+v2*v2)*(1.0/2.0);
	}
	
	if (std::abs(v1) > std::abs(maxSpeed))
		v1 = std::abs(maxSpeed)*MathUtils::sgn<double>(v1);
	//else
	//v1 *= 0.99;
	
	return ((v0*v0)*v2*(1.0/2.0)-v1*(v2*v2)+(v1*v1)*v2+(v2*v2*v2)*(1.0/2.0)+a0*d3*v1-a0*d3*v2+a0*dTotal*v2-v0*v1*v2)/(a0*v1*v2);
}

double KinematicSolver::optimalSpeed2Part(const double accel, const double dTotal, const double v0, const double maxSpeed, double & v1)
{
	double a0 = std::abs(accel);
	double direction = MathUtils::sgn<double>(dTotal);
	v1 = direction*sqrt(a0*dTotal*2.0+v0*v0);
	
	if (std::isnan(v1))
	{
		a0 = -a0;
		v1 = direction*sqrt(a0*dTotal*2.0+v0*v0);
	}
	
	if (std::abs(v1) > std::abs(maxSpeed))
		v1 = std::abs(maxSpeed)*MathUtils::sgn<double>(v1);
	//else
	//v1 *= 0.99;
	
	return (a0*dTotal-v0*v1+(v0*v0)*(1.0/2.0)+(v1*v1)*(1.0/2.0))/(a0*v1);
}


void KinematicSolver::calculatePlan2Part(double absAccel, double tTotal, double dTotal, double v0, PlanSolution & result)
{
	double v1,t0,t1;
	result.valid = false;
	result.t3 = 0;
	result.t2 = 0;
	result.accel1= 0;
	
	double a0 = std::abs(absAccel);
	
	result.i_dTotal = dTotal;
	result.i_v0 = v0;
	result.i_tTotal = tTotal;
	
	double t0_sign = -(sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
	
	if (std::isnan(t0_sign)) return;
	
	if (t0_sign < 0)
	{
		a0 = -a0;
		v1 = v0+sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = (sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal)/a0;
		t1 = -sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
	}
	else
	{
		v1 = v0-sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))+a0*tTotal;
		t0 = -(sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))-a0*tTotal)/a0;
		t1 = sqrt(a0*(dTotal*-2.0+tTotal*v0*2.0+a0*(tTotal*tTotal)))/a0;
	}
	
	result.t0 = t0;
	result.t1 = t1;
	result.accel0 = a0;
	result.travelVelocity = v1;
	result.valid = true;
	
	if (MathDebug) validateSolution(result);
}

void KinematicSolver::calculatePlan(double absAccel, double d3, double tTotal, double dTotal, double v0, double v2, PlanSolution & result)
{
	double v1,t0,t1,t2,t3;
	result.valid = false;
	
	double a0,a1;
	
	if (dTotal > 0)
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
	result.i_dTotal = dTotal;
	result.i_v0 = v0;
	result.i_v2 = v2;
	result.i_tTotal = tTotal;
	
		
	double t0_sign,t2_sign;

	double t1_sign = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2+(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
	(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
	*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2)-(a1*v2*
	(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
	(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);

	if (t1_sign >= 0)
	{
		t0_sign = -(v0+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
		(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
		
		t2_sign = (v2+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
	   *2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
	}
	else
	{
		t0_sign = -(v0-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
				*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
		
		t2_sign = (v2-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
			   *2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
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
		v1 = -((v0*v0)*v2-v2*v2*v2-a0*d3*v2*2.0+a0*dTotal*v2*2.0)/(a0*d3*2.0-v0*v2*2.0+(v2*v2)*2.0-a0*tTotal*v2*2.0);
		t0 = (-v0*(v2*v2)+(v0*v0)*v2*(1.0/2.0)+(v2*v2*v2)*(1.0/2.0)-a0*d3*v0+a0*d3*v2-a0*dTotal*v2+a0*tTotal*v0*v2)/(a0*(a0*d3-v0*v2+v2*v2-a0*tTotal*v2));
		t1 = -(a0*d3-v0*v2+v2*v2-a0*tTotal*v2)/(a0*v2);
		t2 = (-v0*(v2*v2)+(v0*v0)*v2*(1.0/2.0)+(v2*v2*v2)*(1.0/2.0)-a0*tTotal*(v2*v2)+a0*dTotal*v2)/(a0*(a0*d3-v0*v2+v2*v2-a0*tTotal*v2));
		t3 = d3/v2;
	}
	else
	{
		t1 = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2+(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
		(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
		*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2)-(a1*v2*
		(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
		(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);
		
		if (t1 >= 0)
		{
			
			v1 = -(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
							   *2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2);
			
			t0 = -(v0+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*
								   (v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
			
			t2 = (v2+(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)
								  *2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))-a0*(v2*v2)-a0*a1*d3+a1*v0*v2+a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
			
			t3 = d3/v2;
		}
		else //other solution
		{
			v1 = (sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2);
			
			t0 = -(v0-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a0;
			
			t1 = -(a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2-(a0*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*
				(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)
				*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2))/(a0*v2-a1*v2)+
				(a1*v2*(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*
				(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2))/(a0*v2-a1*v2))/(a0*a1*v2);
			
			t2 = (v2-(sqrt(a0*a1*((v0*v0)*(v2*v2)-v0*(v2*v2*v2)*2.0+v2*v2*v2*v2-a0*tTotal*(v2*v2*v2)*2.0+a0*a1*(d3*d3)+a1*d3*(v2*v2)*2.0+a0*dTotal*(v2*v2)*2.0-a1*dTotal*(v2*v2)*2.0-a1*d3*v0*v2*2.0+a1*tTotal*v0*(v2*v2)*2.0+a0*a1*(tTotal*tTotal)*(v2*v2)-a0*a1*d3*tTotal*v2*2.0))+a0*(v2*v2)+a0*a1*d3-a1*v0*v2-a0*a1*tTotal*v2)/(a0*v2-a1*v2))/a1;
			
			t3 = d3/v2;
		}
	}
	
	result.t0 = t0;
	result.t1 = t1;
	result.t2 = t2;
	result.t3 = t3;
	result.accel0 = a0;
	result.accel1 = a1;
	result.travelVelocity = v1;
	result.valid = true;
	
	if (MathDebug) validateSolution(result);
}

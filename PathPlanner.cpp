#include "PathPlanner.hpp"
#include <iostream>

using namespace std;

PathPlanner::PathPlanner(int numChannels, std::vector<double> _velocityConstraints, std::vector<double> _accelerationConstraints, std::vector<double> _jerkConstraints)
{
	this->SamplePeriod = 0.01;
	this->numChannels = numChannels;
	this->velocityConstraints = _velocityConstraints;
	this->accelerationConstraints = _accelerationConstraints;
	this->jerkConstraints = _jerkConstraints;
	
	if (this->velocityConstraints.size() < numChannels) throw logic_error("Velocity constraints are fewer in size than number of channels");
	if (this->accelerationConstraints.size() < numChannels) throw logic_error("Acceleration constraints are fewer in size than number of channels");
	if (this->jerkConstraints.size() < numChannels) throw logic_error("Jerk constraints are fewer in size than the number of channels");
}

void PathPlanner::updateDerivatives(std::vector<Step> & plan)
{
	for (int i=1;i<plan.size();i++)
	{
		Step * s0 = &plan[i];
		Step * s1 = &plan[i-1];
		
		//Find the minimum time that satisfies all channel constraints
		for (int c=0;c<numChannels;c++)
		{
			double dx = s0->Positions[c] - s1->Positions[c];
			double dt = s0->TimeOffset;
			
			double dxdt = dx/dt;
			
			double newTime = 0;
			if (dxdt > velocityConstraints[c])
				newTime = dx/velocityConstraints[c];
			else if (dxdt < -velocityConstraints[c])
				newTime = -dx/velocityConstraints[c];
			else
				continue;
			
			if (newTime > dt)
				s0->TimeOffset = newTime;
//			else cout << "1-New time is less than existing time" << endl;
		}
		
		if (i >= 2)
		{
			Step * s2 = &plan[i-2];
			for (int c=0;c<numChannels;c++)
			{
				double dx2 = (s0->Positions[c] - s1->Positions[c]);
				double dx1 = (s1->Positions[c] - s2->Positions[c]);
				double dt2 = s0->TimeOffset;
				double dt1 = s1->TimeOffset;
				
				if (true)
				{
					double ddxdt =  ((dx2/dt2)-(dx1/dt1))/((dt2+dt1)/2);
					
					//if ((dt2+dt1)/2 < SamplePeriod) ddxdt =  ((dx2/dt2)-(dx1/dt1))/(SamplePeriod);
					
					bool update = true;
					if (ddxdt > accelerationConstraints[c]) ddxdt = accelerationConstraints[c];
					else if (ddxdt < -accelerationConstraints[c]) ddxdt = -accelerationConstraints[c];
					else
						update = false;
					
					if (update)
					{
						double s_dt2 = (-dx1-ddxdt*(dt1*dt1)*(1.0/2.0)-sqrt((dx1*dx1)*4.0+(ddxdt*ddxdt)*(dt1*dt1*dt1*dt1)+ddxdt*(dt1*dt1)*dx1*4.0+ddxdt*(dt1*dt1)*dx2*8.0)*(1.0/2.0))/(ddxdt*dt1);
						
						if (s_dt2 < 0)
							s_dt2 = (-dx1-ddxdt*(dt1*dt1)*(1.0/2.0)+sqrt((dx1*dx1)*4.0+(ddxdt*ddxdt)*(dt1*dt1*dt1*dt1)+ddxdt*(dt1*dt1)*dx1*4.0+ddxdt*(dt1*dt1)*dx2*8.0)*(1.0/2.0))/(ddxdt*dt1);
						
						if (s_dt2 > s0->TimeOffset)
							s0->TimeOffset = s_dt2;
					}
					s0->Accelerations[c] = ddxdt;
				}
				else
				{
					double ddxdt =  ((dx2/dt2)-(dx1/dt1))/((dt2+dt1)/2);
					
					//if ((dt2+dt1)/2 < SamplePeriod) ddxdt =  ((dx2/dt2)-(dx1/dt1))/(SamplePeriod);
					
					bool update = true;
					if (ddxdt > accelerationConstraints[c]) ddxdt = accelerationConstraints[c];
					else if (ddxdt < -accelerationConstraints[c]) ddxdt = -accelerationConstraints[c];
					else
						update = false;
					
					if (update)
					{
						double s_dt2 = (-dx1-ddxdt*(dt1*dt1)*(1.0/2.0)-sqrt((dx1*dx1)*4.0+(ddxdt*ddxdt)*(dt1*dt1*dt1*dt1)+ddxdt*(dt1*dt1)*dx1*4.0+ddxdt*(dt1*dt1)*dx2*8.0)*(1.0/2.0))/(ddxdt*dt1);
						
						if (s_dt2 < 0)
							s_dt2 = (-dx1-ddxdt*(dt1*dt1)*(1.0/2.0)+sqrt((dx1*dx1)*4.0+(ddxdt*ddxdt)*(dt1*dt1*dt1*dt1)+ddxdt*(dt1*dt1)*dx1*4.0+ddxdt*(dt1*dt1)*dx2*8.0)*(1.0/2.0))/(ddxdt*dt1);
						
						if (s_dt2 > s0->TimeOffset)
							s0->TimeOffset = s_dt2;
					}
					s0->Accelerations[c] = ddxdt;
				}
			}
			
			if (i >= 3 && false)
			{
				Step * s3 = &plan[i-3];
				
				for (int c=0;c<numChannels;c++)
				{
					double dx0 = s0->Positions[c] - s1->Positions[c];
					double dx1 = s1->Positions[c] - s2->Positions[c];
					double dx2 = s2->Positions[c] - s3->Positions[c];
					
					double dt0 = s0->TimeOffset, dt1 = s1->TimeOffset, dt2 = s2->TimeOffset;
					
					double ddxdt0 = 2*(dx0/dt0 - dx1/dt1)/(dt0+dt1);
					double ddxdt1 = 2*(dx1/dt1 - dx2/dt2)/(dt1+dt2);

					double dddxdt = 3*(ddxdt0 - ddxdt1)/(dt0+dt1+dt2);
					double idddxdt = dddxdt;
					
					bool update = true;
					if (dddxdt > jerkConstraints[c]) dddxdt = jerkConstraints[c];
					else if (dddxdt < -jerkConstraints[c]) dddxdt = -jerkConstraints[c];
					else update=false;
					
					if (update)
					{
						bool invert = false;
						if (dddxdt < 0)
						{
							//invert = true;
														
							dddxdt = -dddxdt;
							dx0 = dx2*2 - dx0;
							dx1 = dx2*2 - dx1;
							
//							double tmp = dx0;
//							dx0 = dx2;
//							dx2 = tmp;
//							
//							tmp = dt0;
//							dt0 = dt2;
//							dt2 = tmp;
						}
						
						double s_dt0 = pow(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,3.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,3.0)*(-1.0/2.7E1)+(dt1*(dt2*dt2)*dx0*3.0+(dt1*dt1)*dt2*dx0*3.0)/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2)+sqrt(-pow(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,2.0)*(1.0/9.0)-((dt1*dt1)*dx2*-2.0+(dt2*dt2)*dx1*2.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)*(1.0/3.0)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*(2.0/3.0)+dt1*dt2*dx1*4.0+dddxdt*(dt1*dt1*dt1*dt1)*dt2*(1.0/3.0))/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2),3.0)+pow(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,3.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,3.0)*(-1.0/2.7E1)+(dt1*(dt2*dt2)*dx0*3.0+(dt1*dt1)*dt2*dx0*3.0)/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2)+1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0)*((dt1*dt1)*dx2*-6.0+(dt2*dt2)*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*2.0+dt1*dt2*dx1*1.2E1+dddxdt*(dt1*dt1*dt1*dt1)*dt2)*(1.0/6.0),2.0))+1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0)*((dt1*dt1)*dx2*-6.0+(dt2*dt2)*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*2.0+dt1*dt2*dx1*1.2E1+dddxdt*(dt1*dt1*dt1*dt1)*dt2)*(1.0/6.0),1.0/3.0)+(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,2.0)*(1.0/9.0)-((dt1*dt1)*dx2*-2.0+(dt2*dt2)*dx1*2.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)*(1.0/3.0)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*(2.0/3.0)+dt1*dt2*dx1*4.0+dddxdt*(dt1*dt1*dt1*dt1)*dt2*(1.0/3.0))/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2))*1.0/pow(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,3.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,3.0)*(-1.0/2.7E1)+(dt1*(dt2*dt2)*dx0*3.0+(dt1*dt1)*dt2*dx0*3.0)/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2)+sqrt(-pow(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,2.0)*(1.0/9.0)-((dt1*dt1)*dx2*-2.0+(dt2*dt2)*dx1*2.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)*(1.0/3.0)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*(2.0/3.0)+dt1*dt2*dx1*4.0+dddxdt*(dt1*dt1*dt1*dt1)*dt2*(1.0/3.0))/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2),3.0)+pow(1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,3.0)*pow(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0,3.0)*(-1.0/2.7E1)+(dt1*(dt2*dt2)*dx0*3.0+(dt1*dt1)*dt2*dx0*3.0)/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2)+1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0)*((dt1*dt1)*dx2*-6.0+(dt2*dt2)*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*2.0+dt1*dt2*dx1*1.2E1+dddxdt*(dt1*dt1*dt1*dt1)*dt2)*(1.0/6.0),2.0))+1.0/pow(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2,2.0)*(dt1*dx2*-6.0+dt2*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2)*3.0+dddxdt*dt1*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*dt2*2.0)*((dt1*dt1)*dx2*-6.0+(dt2*dt2)*dx1*6.0+dddxdt*(dt1*dt1)*(dt2*dt2*dt2)+dddxdt*(dt1*dt1*dt1)*(dt2*dt2)*2.0+dt1*dt2*dx1*1.2E1+dddxdt*(dt1*dt1*dt1*dt1)*dt2)*(1.0/6.0),1.0/3.0)-(dt1*dx2*-2.0+dt2*dx1*2.0+dddxdt*(dt1*dt1)*(dt2*dt2)+dddxdt*dt1*(dt2*dt2*dt2)*(1.0/3.0)+dddxdt*(dt1*dt1*dt1)*dt2*(2.0/3.0))/(dddxdt*dt1*(dt2*dt2)+dddxdt*(dt1*dt1)*dt2);
  
						if (invert)
						{
							s_dt0 = -s_dt0;
						}
						
						if (std::isnan(s_dt0))
						{
							cout << "non-real solution! initial = " << idddxdt << " dt0=" << dt0 << " dt1=" << dt1 << " dt2=" << dt2 <<
							" dx0=" << dx0 << " dx1=" << dx1 << " dx2=" << dx2 << " dddxdt=" << dddxdt << " i.c=" << i <<"."<<c << endl;
							dddxdt = idddxdt;
						}
						else if (s_dt0 < 0)
						{
							cout << "negative solution! " << s_dt0 << endl;
							dddxdt = idddxdt;
						}
						else if (s_dt0 > s0->TimeOffset)
						{
							if (idddxdt < 0) cout << "real, negative, solution: " << idddxdt << endl;
							s0->TimeOffset = s_dt0;
						}
						else
							cout << "3-New time is less than existing time (" << dt0 << ")" << " s_dt0=" << s_dt0 << endl;
						
						
//This might be faster?
/*
						  t2 = dt1*dt1;
						  t3 = dt2*dt2;
						  t9 = dt1*dx2*6.0;
						  t10 = dt2*dx1*6.0;
						  t11 = dddxdt*t2*t3*3.0;
						  t12 = dddxdt*dt1*dt2*t3;
						  t13 = dddxdt*dt1*dt2*t2*2.0;
						  t4 = -t9+t10+t11+t12+t13;
						  t5 = t4*t4;
						  t6 = dddxdt*dt1*t3;
						  t7 = dddxdt*dt2*t2;
						  t8 = t6+t7;
						  t14 = 1.0/t8;
						  t22 = 1.0/(t8*t8);
						  t23 = dx2*t2*6.0;
						  t24 = dx1*t3*6.0;
						  t25 = dddxdt*dt2*t2*t3;
						  t26 = dddxdt*dt1*t2*t3*2.0;
						  t27 = dt1*dt2*dx1*1.2E1;
						  t28 = t2*t2;
						  t29 = dddxdt*dt2*t28;
						  t30 = -t23+t24+t25+t26+t27+t29;
						  t33 = t5*t22*(1.0/9.0);
						  t34 = t14*t30*(1.0/3.0);
						  t15 = -t33+t34;
						  t16 = t15*t15;
						  t17 = 1.0/(t8*t8*t8);
						  t18 = dt1*dx0*t3*6.0;
						  t19 = dt2*dx0*t2*6.0;
						  t20 = t18+t19;
						  t21 = t14*t20*(1.0/2.0);
						  t32 = t4*t22*t30*(1.0/6.0);
						  t35 = t4*t5*t17*(1.0/2.7E1);
						  t31 = t21+t32-t35;
						  t36 = t33-t34;
						  t37 = t36*t36;
						  t38 = t31*t31;
						  t42 = t36*t37;
						  t39 = t38-t42;
						  t40 = sqrt(t39);
						  t41 = t21+t32-t35+t40;
						  t43 = 1.0/pow(t41,1.0/3.0);
						  t44 = pow(t41,1.0/3.0);
						  t45 = t36*t43;
						  t46 = sqrt(3.0);
						  t47 = t44-t45;
						  A0[0][0] = t45-t4*t14*(1.0/3.0)+pow(t21+t32+sqrt(t38+t15*t16)-t4*t5*t17*(1.0/2.7E1),1.0/3.0);
*/


					}
					s0->Jerks[c] = dddxdt;
				}
			}
		}
//		for (int c=0;c<numChannels;c++)  s1->Velocities[c] = (s1->Positions[c] - s0->Positions[c])/s1->TimeOffset;
	}
}

void PathPlanner::calculateTimeValues(std::vector<Step> &plan)
{
	double totalTime = 0;
	for (int i=0;i<plan.size();i++)
	{
		Step * s1 = &plan[i];
		
		totalTime += s1->TimeOffset;
		s1->Time = totalTime;
		
		if (i>0)
		{
			Step * s0 = &plan[i-1];
			for (int c=0;c<numChannels;c++)
			{
				double dx = s1->Positions[c] - s0->Positions[c];
				double dt = s1->TimeOffset;
				s1->Velocities[c] = dx/dt;
			}
		}
	}
	
	for (int i=2;i<plan.size();i++)
	{
		Step * s2 = &plan[i];
		Step * s1 = &plan[i-1];
		Step * s0 = &plan[i-2];
		
		for (int c=0;c<numChannels;c++)
		{
			double dx1 = (s1->Positions[c] - s0->Positions[c]);
			double dx2 = (s2->Positions[c] - s1->Positions[c]);
			double dx1dt = dx1/s1->TimeOffset;
			double dx2dt = dx2/s2->TimeOffset;
			
			double ddt = (s1->TimeOffset + s2->TimeOffset)/2.0;
			
			double ddx = (dx2dt - dx1dt);
			s2->Accelerations[c] = ddx/ddt;
		}
	}
}

std::vector<Step> PathPlanner::plan(std::vector<Step> & input)
{
	vector<Step> plan = input;
	
	updateDerivatives(plan);
	
	calculateTimeValues(plan);
	
	return plan;
}


std::vector<Step> PathPlanner::interpolate(std::vector<Step> & input, double maxDistance)
{
	vector<Step> interpolated;
	
	interpolated.push_back(input.front());
	for (int i=1;i<input.size();i++)
	{
		Step * prev = &input[i-1];
		Step * s0 = &input[i];
		double intervalCount = 0;
		for (int c=0;c < numChannels; c++)
		{
			double last = prev->Positions[c];
			double next = s0->Positions[c];
			
			intervalCount = std::max((std::abs(next-last)/maxDistance),intervalCount);
		}
				
		for (double n=1; n < intervalCount; n++)
		{
			vector<double> newPos;
			for (int c=0; c<numChannels; c++)
			{
				double last = prev->Positions[c];
				double next = s0->Positions[c];
				double delta = (next-last)/intervalCount;
				newPos.push_back(last + delta*n);
			}
			interpolated.push_back(Step(newPos));
		}
		interpolated.push_back(input[i]);
	}
	
	return interpolated;
}
































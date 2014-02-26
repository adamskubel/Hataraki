#ifndef HATARAKI_MOTION_CONTROL_PATHPLANNER_HPP_
#define HATARAKI_MOTION_CONTROL_PATHPLANNER_HPP_

#include <vector>
#include <stdexcept>
#include <cmath>

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

class PathPlanner {

private:
	std::vector<double> velocityConstraints;
	std::vector<double> accelerationConstraints;
	std::vector<double> jerkConstraints;
	double SamplePeriod;

	void updateDerivatives(std::vector<Step> & plan);
	
	void calculateTimeValues(std::vector<Step> & plan);
	
	bool isMaximumVelocity(double dxdt);
	
	int numChannels;
	
public:
	PathPlanner(int numChannels, std::vector<double> velocityConstraints, std::vector<double> accelerationConstraints, std::vector<double> jerkConstraints);

	std::vector<Step> plan(std::vector<Step> & input);
	
	std::vector<Step> interpolate(std::vector<Step> & input, double maxDistance);
	
};


#endif

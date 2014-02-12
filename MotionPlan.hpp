#ifndef HATARAKI_BASICMOTION_JOINT_MOTION_PLAN_HPP_
#define HATARAKI_BASICMOTION_JOINT_MOTION_PLAN_HPP_

#include <vector>
#include <cmath>

class MotionInterval {
	
public:
	double startSpeed, endSpeed, duration;
	
	MotionInterval(double constantSpeed, double _duration)
	{
		this->startSpeed = constantSpeed;
		this->endSpeed = constantSpeed;
		this->duration = _duration;
	}
	
	MotionInterval(double _startSpeed, double _endSpeed, double _duration) {
		this->startSpeed = _startSpeed;
		this->endSpeed = _endSpeed;
		this->duration = _duration;
	}
	
	double getSpeedAtTime(double time);
	double getPositionAtTime(double time);
	
};

class MotionPlan {
	
public:
	double finalAngle, startAngle;
	timespec startTime, endTime;
	std::vector<MotionInterval> motionIntervals;
		
	double getSpeedAtTime(double planTime);
	double getPositionAtTime(double planTime);
	
	MotionPlan()
	{

	}

	MotionPlan(double targetAngle)
	{		
		startAngle = targetAngle;
		finalAngle = targetAngle;
		motionIntervals.push_back(0,0);
	}
	
	MotionPlan(std::vector<MotionInterval> motionIntervals, double startAngle, timespec startTime) {
		this->motionIntervals = motionIntervals;
		this->startAngle = startAngle;
		this->startTime = startTime;
		this->finalAngle = getPositionAtTime(1000.0); //infinity 

		this->endTime = startTime + 1000; //Need to calculate end time
	}
	
	
};

#endif
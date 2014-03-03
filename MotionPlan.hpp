#ifndef HATARAKI_BASICMOTION_JOINT_MOTION_PLAN_HPP_
#define HATARAKI_BASICMOTION_JOINT_MOTION_PLAN_HPP_

#include <vector>
#include <cmath>
#include <stdexcept>
#include <map>

#include "TimeUtil.hpp"

class MotionInterval {
	

public:
	
	enum Action {
		Travel,
		Start
	};

	double startSpeed, endSpeed, duration;

	Action action;
	
	MotionInterval(double constantSpeed, double _duration);	
	MotionInterval(double _startSpeed, double _endSpeed, double _duration);
	
	double x(double t);
	double dx(double t);
	double ddx(double t);
	
};

class MotionPlan {
	
public:
	double finalAngle, startAngle;
	timespec startTime, endTime;
	std::map<double,double> keyframes;
	
	std::vector<MotionInterval> motionIntervals;
		
	double getPlanDuration();
	
	double x(double t);
	double dx(double t);
	double ddx(double t);

	
	
	MotionPlan()
	{

	}

	MotionPlan(double targetAngle)
	{		
		startAngle = targetAngle;
		finalAngle = targetAngle;
		motionIntervals.push_back(MotionInterval(0,0));
	}
	
//	MotionPlan(std::vector<MotionInterval> motionIntervals, double startAngle, timespec startTime) {
//		this->motionIntervals = motionIntervals;
//		this->startAngle = startAngle;
//		this->startTime = startTime;
//		this->finalAngle = x(1000.0); //infinity 
//
//		TimeUtil::addTime(startTime,getPlanDuration(),endTime);
//	}

	void startNow();
	
	void markKeyframe();
	void markKeyframe(double x);
	
	
};

#endif
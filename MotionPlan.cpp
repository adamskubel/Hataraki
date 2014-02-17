#include "MotionPlan.hpp"

MotionInterval::MotionInterval(double constantSpeed, double _duration)
{
	this->startSpeed = constantSpeed;
	this->endSpeed = constantSpeed;
	this->duration = _duration;

	if (duration < 0) 
		throw std::runtime_error("Motion plan has negative duration");
}

MotionInterval::MotionInterval(double _startSpeed, double _endSpeed, double _duration) {
	this->startSpeed = _startSpeed;
	this->endSpeed = _endSpeed;
	this->duration = _duration;

	if (duration < 0) 
		throw std::runtime_error("Motion plan has negative duration");
}

double MotionPlan::getSpeedAtTime(double planTime)
{
	double intervalStart = 0;
	for (auto it=motionIntervals.begin();it != motionIntervals.end(); it++)
	{
		double intervalEnd = intervalStart+it->duration;
		if (planTime >= intervalStart && planTime < intervalEnd)
		{
			return it->getSpeedAtTime(planTime - intervalStart);
		}
		intervalStart = intervalEnd;
	}
	return motionIntervals.back().endSpeed;
}

double MotionPlan::getPositionAtTime(double planTime)
{
	double position = startAngle, intervalStart = 0;
	
	for (auto it=motionIntervals.begin();it != motionIntervals.end(); it++)
	{
		double intervalEnd = intervalStart+it->duration;
		
		if (planTime >= intervalStart && planTime < intervalEnd)
			position += it->getPositionAtTime(planTime - intervalStart);
		else if (planTime >= intervalStart)
			position += it->getPositionAtTime(it->duration);
		else
			break;
		
		intervalStart = intervalEnd;
	}
	return position;
}

double MotionPlan::getPlanDuration()
{
	double duration = 0;
	for (auto it=motionIntervals.begin();it != motionIntervals.end(); it++)
	{
		duration += it->duration;
	}
	return duration;
}

void MotionPlan::startNow()
{
	TimeUtil::setNow(startTime);
	TimeUtil::addTime(startTime,getPlanDuration(),endTime);
}

double MotionInterval::getSpeedAtTime(double time)
{
	double accel = (endSpeed - startSpeed)/duration;
	return startSpeed + (accel*time);
}

double MotionInterval::getPositionAtTime(double time)
{
	double accel = (endSpeed - startSpeed)/duration;
	return startSpeed * time + (accel * std::pow(time,2))/2.0;
}
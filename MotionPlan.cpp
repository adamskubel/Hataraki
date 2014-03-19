#include "MotionPlan.hpp"


double MotionPlan::x(double t)
{
	double position = startAngle, start = 0;
	
	for (auto interval=motionIntervals.begin();interval != motionIntervals.end(); interval++)
	{
		double end = start+interval->duration;
		
		if (t >= start && t < end)
			position += interval->x(t - start);
		else if (t >= start)
			position += interval->x(interval->duration);
		else
			break;
		
		start = end;
	}
	return position;
}


double MotionPlan::dx(double t)
{
	double start = 0;
	for (auto interval=motionIntervals.begin();interval != motionIntervals.end(); interval++)
	{
		double end = start+interval->duration;
		if (t >= start && t < end)
		{
			return interval->dx(t - start);
		}
		start = end;
	}
	return 0; // motionIntervals.back().endSpeed;
}


double MotionPlan::ddx(double t)
{
	double start = 0;
	for (auto interval=motionIntervals.begin();interval != motionIntervals.end(); interval++)
	{
		double end = start+interval->duration;
		if (t >= start && t < end)
		{
			return interval->ddx(t - start);
		}
		start = end;
	} 
	return 0;
}


double MotionPlan::getPlanDuration()
{
	double duration = 0;
	for (auto interval=motionIntervals.begin();interval != motionIntervals.end(); interval++)
	{
		duration += interval->duration;
	}
	return duration;
}

void MotionPlan::startNow()
{
	TimeUtil::setNow(startTime);
	TimeUtil::addTime(startTime,getPlanDuration(),endTime);
	if (TimeUtil::getTimeDelta(startTime, endTime) < 0)
		throw std::runtime_error("Plan starts before ending");
}

void MotionPlan::markKeyframe()
{
	double t = getPlanDuration();
	keyframes.insert(std::make_pair(t,x(t)));
}

void MotionPlan::markKeyframe(double overrideX)
{
	double t = getPlanDuration();
	keyframes.insert(std::make_pair(t,overrideX));
}

MotionInterval::MotionInterval(double constantSpeed, double _duration)
{
	this->startSpeed = constantSpeed;
	this->endSpeed = constantSpeed;
	this->duration = _duration;
	this->action = Action::Travel;

	if (duration < 0)
		throw std::runtime_error("Motion plan has negative duration");
}

MotionInterval::MotionInterval(double _startSpeed, double _endSpeed, double _duration) {
	this->startSpeed = _startSpeed;
	this->endSpeed = _endSpeed;
	this->duration = _duration;
	this->action = Action::Travel;

	if (duration < 0)
		throw std::runtime_error("Motion plan has negative duration");
}

double MotionInterval::ddx(double time)
{
	return (endSpeed - startSpeed)/duration;
}

double MotionInterval::dx(double time)
{
	double accel = (endSpeed - startSpeed)/duration;
	return startSpeed + (accel*time);
}

double MotionInterval::x(double time)
{
	if (action == Action::Travel)
	{
		double accel = (endSpeed - startSpeed)/duration;
		return startSpeed * time + (accel * std::pow(time,2))/2.0;
	}
	else
	{
		return 0;
	}
}

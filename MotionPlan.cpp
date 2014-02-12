#include "MotionPlan.hpp"


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
			position += getPositionAtTime(it->getPositionAtTime(planTime - intervalStart));
		else if (planTime >= intervalStart)
			position += getPositionAtTime(it->duration);
		else
			break;
		
		intervalStart = intervalEnd;
	}
	return position;
}

double MotionInterval::getSpeedAtTime(double time)
{
	double accel = endSpeed - startSpeed;
	return startSpeed + (accel*time)/duration;
}

double MotionInterval::getPositionAtTime(double time)
{
	double accel = endSpeed - startSpeed;
	return startSpeed * time + (accel * std::pow(time,2))/2.0;
}